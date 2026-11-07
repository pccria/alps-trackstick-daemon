/**
 * @file calibrate.c
 * @brief Implementation of guided 4-phase physical calibration and interactive Control Center.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "features/calibrate.h"
#include "io/device.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <sys/epoll.h>
#include <sys/wait.h>
#include <linux/input.h>

static volatile sig_atomic_t g_cal_running = 1;

static void cal_sig_handler(int sig) {
    (void)sig;
    g_cal_running = 0;
}

static uint64_t cal_get_time_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0) {
        return 0;
    }
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
}

static void cal_wait_for_user(const char *prompt) {
    if (!isatty(STDIN_FILENO)) return;
    if (prompt) {
        printf("%s", prompt);
        fflush(stdout);
    }
    char line[128];
    if (!fgets(line, sizeof(line), stdin)) return;
}

static int cal_prompt_choice(const char *prompt, const char *valid_chars, char default_choice) {
    if (!isatty(STDIN_FILENO)) return default_choice;
    printf("%s", prompt);
    fflush(stdout);
    char line[128];
    if (!fgets(line, sizeof(line), stdin)) return default_choice;
    char *p = line;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0' || *p == '\n' || *p == '\r') return default_choice;
    char c = (char)tolower((unsigned char)*p);
    if (valid_chars && strchr(valid_chars, c)) return c;
    return default_choice;
}

void alps_calibrate_tracker_init(alps_calibrate_tracker_t *tracker) {
    if (!tracker) return;
    memset(tracker, 0, sizeof(*tracker));
    tracker->min_tap_z = 255;
}

void alps_calibrate_profile_init(alps_guided_profile_t *prof) {
    if (!prof) return;
    memset(prof, 0, sizeof(*prof));
    prof->single_tap_min_peak = 255;
    prof->rec_tap1_pressure = ALPS_DEFAULT_TAP1_PRESSURE;
    prof->rec_tap2_pressure = ALPS_DEFAULT_TAP2_REBOUND_PRESSURE;
    prof->rec_release_pressure = ALPS_DEFAULT_RELEASE_PRESSURE;
    prof->rec_tap_duration_ms = ALPS_DEFAULT_MAX_TAP_DURATION_MS;
    prof->rec_double_window_ms = ALPS_DEFAULT_DOUBLE_TAP_WINDOW_MS;
    prof->rec_tap_motion = ALPS_DEFAULT_MAX_TAP_MOTION;
}

int alps_calibrate_tracker_feed_event(alps_calibrate_tracker_t *tracker,
                                      const alps_event_t *ev,
                                      uint64_t now_ms) {
    if (!tracker || !ev) return 0;

    int tap_completed = 0;

    if (ev->type == ALPS_EV_PRESSURE) {
        tracker->current_z = ev->value;

        if (ev->value > 0) {
            if (tracker->active_press_start_ms == 0) {
                tracker->active_press_start_ms = now_ms;
                tracker->active_peak_z = ev->value;
                tracker->active_motion_counts = 0;
            } else if (ev->value > tracker->active_peak_z) {
                tracker->active_peak_z = ev->value;
            }
        } else {
            /* Pressure returned to 0: contact unload */
            if (tracker->active_press_start_ms > 0) {
                uint64_t duration_ms = now_ms >= tracker->active_press_start_ms
                                           ? (now_ms - tracker->active_press_start_ms)
                                           : 0;
                /* Valid impulse tap: 15ms to 350ms, peak >= 8 counts */
                if (duration_ms >= 15 && duration_ms <= 350 && tracker->active_peak_z >= 8) {
                    tracker->tap_count++;
                    tracker->tap_peak_sum += (uint64_t)tracker->active_peak_z;
                    tracker->tap_duration_sum += duration_ms;
                    if (tracker->active_peak_z > tracker->max_tap_z) {
                        tracker->max_tap_z = tracker->active_peak_z;
                    }
                    if (tracker->active_peak_z < tracker->min_tap_z) {
                        tracker->min_tap_z = tracker->active_peak_z;
                    }
                    tap_completed = 1;
                }
                tracker->active_press_start_ms = 0;
            }
        }
    } else if (ev->type == ALPS_EV_REL_X) {
        tracker->current_dx = ev->value;
        if (tracker->active_press_start_ms > 0) {
            tracker->active_motion_counts += abs(ev->value);
        }
    } else if (ev->type == ALPS_EV_REL_Y) {
        tracker->current_dy = ev->value;
        if (tracker->active_press_start_ms > 0) {
            tracker->active_motion_counts += abs(ev->value);
        }
    }

    return tap_completed;
}

void alps_calibrate_format_bar(int z, int max_z, int width, char *out_buf, size_t out_max) {
    if (!out_buf || out_max < 3) return;
    if (width <= 0) width = 30;
    if (max_z <= 0) max_z = 100;
    if (z < 0) z = 0;
    if (z > max_z) z = max_z;

    int filled = (z * width) / max_z;
    if (filled > width) filled = width;

    size_t pos = 0;
    if (pos < out_max) out_buf[pos++] = '[';

    for (int i = 0; i < width && pos + 2 < out_max; i++) {
        if (i < filled - 1) {
            out_buf[pos++] = '=';
        } else if (i == filled - 1) {
            out_buf[pos++] = '>';
        } else {
            out_buf[pos++] = ' ';
        }
    }

    if (pos < out_max) out_buf[pos++] = ']';
    out_buf[pos < out_max ? pos : out_max - 1] = '\0';
}

void alps_calibrate_compute_recommendation(const alps_calibrate_tracker_t *tracker,
                                          int *out_tap1,
                                          int *out_tap2,
                                          int *out_release) {
    if (!out_tap1 || !out_tap2 || !out_release) return;

    if (!tracker || tracker->tap_count < 2) {
        *out_tap1 = ALPS_DEFAULT_TAP1_PRESSURE;
        *out_tap2 = ALPS_DEFAULT_TAP2_REBOUND_PRESSURE;
        *out_release = ALPS_DEFAULT_RELEASE_PRESSURE;
        return;
    }

    int avg_peak = (int)(tracker->tap_peak_sum / (uint64_t)tracker->tap_count);

    /* Tap 1 activation floor: 70% of average peak tap force */
    int t1 = (avg_peak * 70) / 100;
    if (t1 < 14) t1 = 14;
    if (t1 > 90) t1 = 90;

    /* Tap 2 rebound: 68% of Tap 1 floor */
    int t2 = (t1 * 68) / 100;
    if (t2 < 10) t2 = 10;

    /* Mechanical release baseline: 45% of Tap 1 floor */
    int rel = (t1 * 45) / 100;
    if (rel < 5) rel = 5;

    *out_tap1 = t1;
    *out_tap2 = t2;
    *out_release = rel;
}

void alps_calibrate_compute_guided_recommendations(alps_guided_profile_t *prof) {
    if (!prof) return;

    /* If insufficient taps were recorded, default to factory profile */
    if (prof->single_tap_count < 2 || prof->single_tap_avg_peak <= 0) {
        prof->rec_tap1_pressure = ALPS_DEFAULT_TAP1_PRESSURE;
        prof->rec_tap2_pressure = ALPS_DEFAULT_TAP2_REBOUND_PRESSURE;
        prof->rec_release_pressure = ALPS_DEFAULT_RELEASE_PRESSURE;
        prof->rec_tap_duration_ms = ALPS_DEFAULT_MAX_TAP_DURATION_MS;
        prof->rec_double_window_ms = ALPS_DEFAULT_DOUBLE_TAP_WINDOW_MS;
        prof->rec_tap_motion = ALPS_DEFAULT_MAX_TAP_MOTION;
        return;
    }

    int noise_max = prof->noise_floor_max_z;

    /* Tap 1 activation threshold: ~70% of average peak strike force */
    int t1 = (prof->single_tap_avg_peak * 70) / 100;

    /* Do not exceed the lightest recorded tap if reasonable */
    if (prof->single_tap_min_peak > 0 && t1 > prof->single_tap_min_peak) {
        t1 = prof->single_tap_min_peak;
    }

    /* Must sit safely above resting noise floor to prevent false triggers */
    if (t1 < noise_max + 8) {
        t1 = noise_max + 8;
    }

    /* Safe physical bounds [14, 90] */
    if (t1 < 14) t1 = 14;
    if (t1 > 90) t1 = 90;

    /* Tap 2 rebound threshold: derived from measured second strike if available */
    int t2;
    if (prof->double_tap_count >= 1 && prof->double_tap2_avg_peak > 0) {
        t2 = (prof->double_tap2_avg_peak * 70) / 100;
    } else {
        t2 = (t1 * 68) / 100;
    }

    if (t2 < noise_max + 5) {
        t2 = noise_max + 5;
    }
    if (t2 >= t1) {
        t2 = t1 - 2;
    }
    if (t2 < 10) t2 = 10;

    /* Release baseline: contact unload detection */
    int rel = (t1 * 45) / 100;
    if (rel < noise_max + 2) {
        rel = noise_max + 2;
    }
    if (rel >= t2) {
        rel = t2 - 2;
    }
    if (rel < 5) rel = 5;

    /* Contact dwell duration: average dwell + 40% margin, clamped [120ms, 350ms] */
    uint32_t dur = prof->single_tap_avg_dur_ms > 0
                       ? prof->single_tap_avg_dur_ms + (prof->single_tap_avg_dur_ms * 40 / 100)
                       : ALPS_DEFAULT_MAX_TAP_DURATION_MS;
    if (dur < 120) dur = 120;
    if (dur > 350) dur = 350;

    /* Double tap window: average gap + 50% cushion, clamped [180ms, 450ms] */
    uint32_t win = prof->double_tap_avg_gap_ms > 0
                       ? prof->double_tap_avg_gap_ms + (prof->double_tap_avg_gap_ms * 50 / 100)
                       : ALPS_DEFAULT_DOUBLE_TAP_WINDOW_MS;
    if (win < 180) win = 180;
    if (win > 450) win = 450;

    /* Tap motion tolerance: derived from navigation shear if available, else default */
    int mot = ALPS_DEFAULT_MAX_TAP_MOTION;
    if (prof->steer_max_motion > 0) {
        mot = prof->steer_max_motion / 3;
        if (mot < 8) mot = 8;
        if (mot > 20) mot = 20;
    }

    prof->rec_tap1_pressure = t1;
    prof->rec_tap2_pressure = t2;
    prof->rec_release_pressure = rel;
    prof->rec_tap_duration_ms = dur;
    prof->rec_double_window_ms = win;
    prof->rec_tap_motion = mot;
}

int alps_calibrate_save_config(const char *config_path, const alps_guided_profile_t *prof) {
    if (!prof) return -1;
    const char *target = (config_path && config_path[0] != '\0') ? config_path : ALPS_DEFAULT_CONFIG_PATH;

    char tmp_path[512];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", target);

    FILE *in = fopen(target, "r");
    FILE *out = fopen(tmp_path, "w");
    if (!out) {
        if (in) fclose(in);
        if (errno == EACCES || errno == EPERM) {
            return -2;
        }
        return -1;
    }

    if (!in) {
        time_t now = time(NULL);
        char time_str[64] = {0};
        struct tm tm_buf;
        if (localtime_r(&now, &tm_buf)) {
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);
        }

        fprintf(out, "# ALPS DualPoint Trackstick Daemon Configuration\n");
        fprintf(out, "# Generated by alps-stick --calibrate on %s\n\n", time_str[0] ? time_str : "auto");
        fprintf(out, "[general]\n");
        fprintf(out, "verbose = false\n\n");
        fprintf(out, "[tap]\n");
        fprintf(out, "tap1_pressure = %d\n", prof->rec_tap1_pressure);
        fprintf(out, "tap2_pressure = %d\n", prof->rec_tap2_pressure);
        fprintf(out, "release_pressure = %d\n", prof->rec_release_pressure);
        fprintf(out, "tap_duration_ms = %u\n", prof->rec_tap_duration_ms);
        fprintf(out, "tap_motion = %d\n", prof->rec_tap_motion);
        fprintf(out, "double_tap_window_ms = %u\n", prof->rec_double_window_ms);
        fprintf(out, "anti_drift = true\n\n");
        fprintf(out, "[scroll]\n");
        fprintf(out, "enabled = true\n");
        fprintf(out, "hires_multiplier = %d\n", ALPS_DEFAULT_SCROLL_HIRES_MULTIPLIER);
        fprintf(out, "progressive = true\n");
        fprintf(out, "accel_factor = %d\n", ALPS_DEFAULT_SCROLL_ACCEL_FACTOR);
        fprintf(out, "axis_lock = true\n");
        fprintf(out, "counter_drift = true\n");

        fclose(out);
        if (rename(tmp_path, target) != 0) {
            unlink(tmp_path);
            if (errno == EACCES || errno == EPERM) return -2;
            return -1;
        }
        return 0;
    }

    char line[512];
    int in_tap_sec = 0;
    int tap_sec_found = 0;
    int written_tap1 = 0;
    int written_tap2 = 0;
    int written_release = 0;
    int written_dur = 0;
    int written_mot = 0;
    int written_win = 0;

    while (fgets(line, sizeof(line), in)) {
        char copy[512];
        snprintf(copy, sizeof(copy), "%s", line);

        char *p = copy;
        while (isspace((unsigned char)*p)) p++;

        if (*p == '[') {
            char *end_b = strchr(p, ']');
            if (end_b) {
                if (in_tap_sec) {
                    if (!written_tap1) fprintf(out, "tap1_pressure = %d\n", prof->rec_tap1_pressure);
                    if (!written_tap2) fprintf(out, "tap2_pressure = %d\n", prof->rec_tap2_pressure);
                    if (!written_release) fprintf(out, "release_pressure = %d\n", prof->rec_release_pressure);
                    if (!written_dur) fprintf(out, "tap_duration_ms = %u\n", prof->rec_tap_duration_ms);
                    if (!written_mot) fprintf(out, "tap_motion = %d\n", prof->rec_tap_motion);
                    if (!written_win) fprintf(out, "double_tap_window_ms = %u\n", prof->rec_double_window_ms);
                    in_tap_sec = 0;
                }

                *end_b = '\0';
                char *sec_name = p + 1;
                while (isspace((unsigned char)*sec_name)) sec_name++;
                char *end_s = sec_name + strlen(sec_name) - 1;
                while (end_s > sec_name && isspace((unsigned char)*end_s)) *end_s-- = '\0';

                if (strcasecmp(sec_name, "tap") == 0) {
                    in_tap_sec = 1;
                    tap_sec_found = 1;
                }
            }
            fputs(line, out);
            continue;
        }

        if (in_tap_sec && *p != '#' && *p != ';') {
            char *eq = strchr(p, '=');
            if (!eq) eq = strchr(p, ':');
            if (eq) {
                *eq = '\0';
                char *k = p;
                while (isspace((unsigned char)*k)) k++;
                char *end_k = k + strlen(k) - 1;
                while (end_k > k && isspace((unsigned char)*end_k)) *end_k-- = '\0';

                if (strcasecmp(k, "tap1_pressure") == 0) {
                    fprintf(out, "tap1_pressure = %d\n", prof->rec_tap1_pressure);
                    written_tap1 = 1;
                    continue;
                }
                if (strcasecmp(k, "tap2_pressure") == 0 || strcasecmp(k, "tap2_rebound_pressure") == 0) {
                    fprintf(out, "tap2_pressure = %d\n", prof->rec_tap2_pressure);
                    written_tap2 = 1;
                    continue;
                }
                if (strcasecmp(k, "release_pressure") == 0) {
                    fprintf(out, "release_pressure = %d\n", prof->rec_release_pressure);
                    written_release = 1;
                    continue;
                }
                if (strcasecmp(k, "tap_duration_ms") == 0 || strcasecmp(k, "max_tap_duration_ms") == 0 || strcasecmp(k, "tap_duration") == 0) {
                    fprintf(out, "tap_duration_ms = %u\n", prof->rec_tap_duration_ms);
                    written_dur = 1;
                    continue;
                }
                if (strcasecmp(k, "tap_motion") == 0 || strcasecmp(k, "max_tap_motion") == 0) {
                    fprintf(out, "tap_motion = %d\n", prof->rec_tap_motion);
                    written_mot = 1;
                    continue;
                }
                if (strcasecmp(k, "double_tap_window_ms") == 0 || strcasecmp(k, "double_window") == 0) {
                    fprintf(out, "double_tap_window_ms = %u\n", prof->rec_double_window_ms);
                    written_win = 1;
                    continue;
                }
            }
        }

        fputs(line, out);
    }

    if (in_tap_sec) {
        if (!written_tap1) fprintf(out, "tap1_pressure = %d\n", prof->rec_tap1_pressure);
        if (!written_tap2) fprintf(out, "tap2_pressure = %d\n", prof->rec_tap2_pressure);
        if (!written_release) fprintf(out, "release_pressure = %d\n", prof->rec_release_pressure);
        if (!written_dur) fprintf(out, "tap_duration_ms = %u\n", prof->rec_tap_duration_ms);
        if (!written_mot) fprintf(out, "tap_motion = %d\n", prof->rec_tap_motion);
        if (!written_win) fprintf(out, "double_tap_window_ms = %u\n", prof->rec_double_window_ms);
    } else if (!tap_sec_found) {
        fprintf(out, "\n[tap]\n");
        fprintf(out, "tap1_pressure = %d\n", prof->rec_tap1_pressure);
        fprintf(out, "tap2_pressure = %d\n", prof->rec_tap2_pressure);
        fprintf(out, "release_pressure = %d\n", prof->rec_release_pressure);
        fprintf(out, "tap_duration_ms = %u\n", prof->rec_tap_duration_ms);
        fprintf(out, "tap_motion = %d\n", prof->rec_tap_motion);
        fprintf(out, "double_tap_window_ms = %u\n", prof->rec_double_window_ms);
    }

    fclose(in);
    fclose(out);

    if (rename(tmp_path, target) != 0) {
        unlink(tmp_path);
        if (errno == EACCES || errno == EPERM) return -2;
        return -1;
    }
    return 0;
}

int alps_calibrate_run(const alps_config_t *cfg) {
    char dev_path[512] = {0};
    char dev_name[256] = {0};

    int stick_fd = alps_device_find_stick(dev_path, sizeof(dev_path), dev_name, sizeof(dev_name));
    if (stick_fd < 0) {
        fprintf(stderr, "\nError: ALPS DualPoint Stick not found in /dev/input/event*!\n");
        fprintf(stderr, "Ensure kernel driver (psmouse/alps) is loaded and permissions allow reading.\n\n");
        return 1;
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = cal_sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        perror("epoll_create1 failed");
        close(stick_fd);
        return 1;
    }

    struct epoll_event ev_sub;
    memset(&ev_sub, 0, sizeof(ev_sub));
    ev_sub.events = EPOLLIN;
    ev_sub.data.fd = stick_fd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, stick_fd, &ev_sub) < 0) {
        perror("epoll_ctl failed");
        close(epfd);
        close(stick_fd);
        return 1;
    }

    const char *target_config_path = (cfg && cfg->config_path[0] != '\0')
                                         ? cfg->config_path
                                         : ALPS_DEFAULT_CONFIG_PATH;

restart_wizard:
    g_cal_running = 1;

    printf("\n");
    printf("==============================================================================\n");
    printf("  ALPS Trackstick Guided 4-Phase Physical Calibration Wizard (v%s)\n", ALPS_DAEMON_VERSION);
    printf("==============================================================================\n");
    printf("Connected Hardware: %s (%s)\n", dev_name, dev_path);
    printf("Target Config File: %s\n\n", target_config_path);
    printf("This wizard customizes tap detection and kinetic parameters specifically\n");
    printf("for your physical trackstick hardware and natural clicking rhythm.\n\n");
    printf("The calibration consists of 4 distinct steps:\n");
    printf("  Step 1: Resting baseline noise floor (2.0 seconds hands-off)\n");
    printf("  Step 2: Natural single taps (5 clicks)\n");
    printf("  Step 3: Double-click cadence & rebound (3 double clicks)\n");
    printf("  Step 4: Steering shear profile (steer in circles for 3 seconds)\n\n");
    printf("IMPORTANT:\n");
    printf("  • Take your time. Each step will wait for you to read instructions first.\n");
    printf("  • No timed countdown will start until you explicitly press [Enter].\n");
    printf("  • You can retry any individual step if you want to re-sample.\n");
    printf("  • Press Ctrl+C at any time to cancel without modifying configuration.\n");
    printf("==============================================================================\n");
    fflush(stdout);

    cal_wait_for_user("Press [Enter] when ready to begin Step 1... ");

    alps_guided_profile_t prof;
    alps_calibrate_profile_init(&prof);

    struct input_event raw_events[32];
    char bar_buf[64];

    /* =========================================================================
     * Step 1/4: Resting Baseline & Noise Floor
     * ========================================================================= */
phase1_retry:
    if (!g_cal_running) goto cleanup_exit;

    printf("\n------------------------------------------------------------------------------\n");
    printf("Step 1 of 4: Resting Baseline & Noise Floor\n");
    printf("------------------------------------------------------------------------------\n");
    printf("INSTRUCTIONS:\n");
    printf("  1. Completely remove your hand and fingers from the trackstick.\n");
    printf("  2. Keep the laptop steady on a desk or table.\n");
    printf("  3. When you press [Enter], the sensor will be sampled for 2.0 seconds\n");
    printf("     to record quiescent noise levels and establish the zero baseline.\n\n");
    fflush(stdout);

    cal_wait_for_user("--> Release the trackstick completely, then press [Enter] to sample: ");
    printf("Sampling resting baseline for 2.0 seconds...\n");
    fflush(stdout);

    uint64_t p1_start_ms = cal_get_time_ms();
    uint64_t last_render_ms = 0;
    uint64_t noise_z_sum = 0;
    uint32_t noise_samples = 0;
    int cur_z = 0;
    prof.noise_floor_max_z = 0;

    while (g_cal_running) {
        uint64_t now_ms = cal_get_time_ms();
        if (now_ms - p1_start_ms >= 2000) {
            break;
        }

        struct epoll_event ep_ev[1];
        int nfds = epoll_wait(epfd, ep_ev, 1, 50);
        if (nfds > 0) {
            ssize_t bytes = read(stick_fd, raw_events, sizeof(raw_events));
            if (bytes > 0) {
                size_t count = (size_t)bytes / sizeof(struct input_event);
                for (size_t i = 0; i < count; i++) {
                    if (raw_events[i].type == EV_ABS && raw_events[i].code == ABS_PRESSURE) {
                        cur_z = raw_events[i].value;
                        if (cur_z > 14) {
                            /* Contact disturbance detected: notify and restart 2s timer */
                            printf("\r\033[K[!] Contact detected (Z=%d)! Please release fingers completely. Resetting timer...", cur_z);
                            fflush(stdout);
                            p1_start_ms = now_ms;
                            noise_z_sum = 0;
                            noise_samples = 0;
                            prof.noise_floor_max_z = 0;
                            continue;
                        }
                        noise_z_sum += (uint64_t)cur_z;
                        noise_samples++;
                        if (cur_z > prof.noise_floor_max_z) {
                            prof.noise_floor_max_z = cur_z;
                        }
                    }
                }
            }
        }

        if (now_ms - last_render_ms >= 50) {
            last_render_ms = now_ms;
            uint64_t elapsed = now_ms - p1_start_ms;
            int pct = (int)((elapsed * 100) / 2000);
            if (pct > 100) pct = 100;
            alps_calibrate_format_bar((int)elapsed, 2000, 24, bar_buf, sizeof(bar_buf));
            printf("\r\033[K  %s %3d%%  (Z=%d, Peak Noise: %d)",
                   bar_buf, pct, cur_z, prof.noise_floor_max_z);
            fflush(stdout);
        }
    }

    if (!g_cal_running) goto cleanup_exit;

    prof.noise_floor_z = noise_samples > 0 ? (int)(noise_z_sum / noise_samples) : 0;
    printf("\r\033[K  ✓ Step 1 Complete: Resting Z=%d (Transient Noise Floor Peak=%d)\n\n",
           prof.noise_floor_z, prof.noise_floor_max_z);
    fflush(stdout);

    int rep1 = cal_prompt_choice("Accept baseline [Enter] or [r]etry Step 1? [Y/r]: ", "yr", 'y');
    if (rep1 == 'r') goto phase1_retry;

    cal_wait_for_user("\nPress [Enter] to continue to Step 2...");

    /* =========================================================================
     * Step 2/4: Natural Single Taps (5 repetitions)
     * ========================================================================= */
phase2_retry:
    if (!g_cal_running) goto cleanup_exit;

    prof.single_tap_count = 0;
    prof.single_tap_min_peak = 255;
    prof.single_tap_max_peak = 0;
    uint64_t single_peak_sum = 0;
    uint64_t single_dur_sum = 0;
    uint64_t contact_start_ms = 0;
    int active_peak_z = 0;
    int active_shear = 0;
    cur_z = 0;

    printf("\n------------------------------------------------------------------------------\n");
    printf("Step 2 of 4: Natural Single Taps\n");
    printf("------------------------------------------------------------------------------\n");
    printf("INSTRUCTIONS:\n");
    printf("  1. Tap the trackstick naturally 5 times with your index finger.\n");
    printf("  2. Click with your normal force, like clicking a link in a browser.\n");
    printf("  3. The live meter below will show each tap as it registers.\n\n");
    fflush(stdout);

    cal_wait_for_user("--> Place finger over trackstick and press [Enter] to start tapping: ");
    printf("Waiting for 5 natural taps...\n");
    fflush(stdout);

    while (g_cal_running && prof.single_tap_count < 5) {
        struct epoll_event ep_ev[1];
        int nfds = epoll_wait(epfd, ep_ev, 1, 50);
        uint64_t now_ms = cal_get_time_ms();

        if (nfds > 0) {
            ssize_t bytes = read(stick_fd, raw_events, sizeof(raw_events));
            if (bytes > 0) {
                size_t count = (size_t)bytes / sizeof(struct input_event);
                for (size_t i = 0; i < count; i++) {
                    if (raw_events[i].type == EV_ABS && raw_events[i].code == ABS_PRESSURE) {
                        cur_z = raw_events[i].value;
                        if (cur_z > prof.noise_floor_max_z + 3) {
                            if (contact_start_ms == 0) {
                                contact_start_ms = now_ms;
                                active_peak_z = cur_z;
                                active_shear = 0;
                            } else if (cur_z > active_peak_z) {
                                active_peak_z = cur_z;
                            }
                        } else if (cur_z <= prof.noise_floor_max_z + 2) {
                            if (contact_start_ms > 0) {
                                uint64_t dur = now_ms >= contact_start_ms ? (now_ms - contact_start_ms) : 0;
                                if (dur >= 15 && dur <= 400 && active_peak_z >= prof.noise_floor_max_z + 6) {
                                    prof.single_tap_count++;
                                    single_peak_sum += (uint64_t)active_peak_z;
                                    single_dur_sum += dur;
                                    if (active_peak_z > prof.single_tap_max_peak) prof.single_tap_max_peak = active_peak_z;
                                    if (active_peak_z < prof.single_tap_min_peak) prof.single_tap_min_peak = active_peak_z;

                                    printf("\r\033[K  [Tap %d/5] Peak force: %d, Duration: %ums, Shear: %d -> OK\n",
                                           prof.single_tap_count, active_peak_z, (unsigned)dur, active_shear);
                                    fflush(stdout);
                                }
                                contact_start_ms = 0;
                            }
                        }
                    } else if (raw_events[i].type == EV_REL) {
                        if (contact_start_ms > 0) {
                            active_shear += abs(raw_events[i].value);
                        }
                    }
                }
            }
        }

        if (now_ms - last_render_ms >= 50) {
            last_render_ms = now_ms;
            alps_calibrate_format_bar(cur_z, 80, 24, bar_buf, sizeof(bar_buf));
            printf("\r\033[K  %s Z=%-3d  [Tap %d/5] (Tap naturally on trackstick...)",
                   bar_buf, cur_z, prof.single_tap_count + 1);
            fflush(stdout);
        }
    }

    if (!g_cal_running) goto cleanup_exit;

    prof.single_tap_avg_peak = (int)(single_peak_sum / (uint64_t)prof.single_tap_count);
    prof.single_tap_avg_dur_ms = (uint32_t)(single_dur_sum / (uint64_t)prof.single_tap_count);
    printf("  ✓ Step 2 Complete: 5 taps recorded (Avg Peak=%d, Min=%d, Max=%d, Avg Dwell=%ums)\n\n",
           prof.single_tap_avg_peak, prof.single_tap_min_peak, prof.single_tap_max_peak,
           prof.single_tap_avg_dur_ms);
    fflush(stdout);

    int rep2 = cal_prompt_choice("Keep these taps [Enter] or [r]etry Step 2? [Y/r]: ", "yr", 'y');
    if (rep2 == 'r') goto phase2_retry;

    cal_wait_for_user("\nPress [Enter] to continue to Step 3...");

    /* =========================================================================
     * Step 3/4: Double-Click Cadence & Rebound (3 repetitions)
     * ========================================================================= */
phase3_retry:
    if (!g_cal_running) goto cleanup_exit;

    prof.double_tap_count = 0;
    int phase3_sub = 0;
    int t1_peak = 0;
    uint64_t t1_end_ms = 0;
    uint64_t d_t1_sum = 0;
    uint64_t d_t2_sum = 0;
    uint64_t d_gap_sum = 0;
    contact_start_ms = 0;
    active_peak_z = 0;
    cur_z = 0;

    printf("\n------------------------------------------------------------------------------\n");
    printf("Step 3 of 4: Double-Click Cadence & Rebound\n");
    printf("------------------------------------------------------------------------------\n");
    printf("INSTRUCTIONS:\n");
    printf("  1. Perform 3 natural double-clicks (tap-tap) at your normal double-click rhythm.\n");
    printf("  2. This measures your double-click speed and the rebound force difference\n");
    printf("     between the first strike and second rebound strike.\n\n");
    fflush(stdout);

    cal_wait_for_user("--> Press [Enter] when ready to perform double-clicks: ");
    printf("Waiting for 3 double-clicks...\n");
    fflush(stdout);

    while (g_cal_running && prof.double_tap_count < 3) {
        struct epoll_event ep_ev[1];
        int nfds = epoll_wait(epfd, ep_ev, 1, 50);
        uint64_t now_ms = cal_get_time_ms();

        if (phase3_sub == 1 && now_ms - t1_end_ms > 500) {
            phase3_sub = 0;
        }

        if (nfds > 0) {
            ssize_t bytes = read(stick_fd, raw_events, sizeof(raw_events));
            if (bytes > 0) {
                size_t count = (size_t)bytes / sizeof(struct input_event);
                for (size_t i = 0; i < count; i++) {
                    if (raw_events[i].type == EV_ABS && raw_events[i].code == ABS_PRESSURE) {
                        cur_z = raw_events[i].value;
                        if (cur_z > prof.noise_floor_max_z + 3) {
                            if (contact_start_ms == 0) {
                                contact_start_ms = now_ms;
                                active_peak_z = cur_z;
                            } else if (cur_z > active_peak_z) {
                                active_peak_z = cur_z;
                            }
                        } else if (cur_z <= prof.noise_floor_max_z + 2) {
                            if (contact_start_ms > 0) {
                                uint64_t dur = now_ms >= contact_start_ms ? (now_ms - contact_start_ms) : 0;
                                if (dur >= 15 && dur <= 400 && active_peak_z >= prof.noise_floor_max_z + 6) {
                                    if (phase3_sub == 0) {
                                        t1_peak = active_peak_z;
                                        t1_end_ms = now_ms;
                                        phase3_sub = 1;
                                    } else if (phase3_sub == 1) {
                                        uint64_t gap = contact_start_ms >= t1_end_ms ? (contact_start_ms - t1_end_ms) : 0;
                                        int t2_peak = active_peak_z;
                                        prof.double_tap_count++;
                                        d_t1_sum += (uint64_t)t1_peak;
                                        d_t2_sum += (uint64_t)t2_peak;
                                        d_gap_sum += gap;

                                        printf("\r\033[K  [Double %d/3] Tap 1: %d, Rebound: %d, Inter-tap Gap: %ums -> OK\n",
                                               prof.double_tap_count, t1_peak, t2_peak, (unsigned)gap);
                                        fflush(stdout);
                                        phase3_sub = 0;
                                    }
                                }
                                contact_start_ms = 0;
                            }
                        }
                    }
                }
            }
        }

        if (now_ms - last_render_ms >= 50) {
            last_render_ms = now_ms;
            alps_calibrate_format_bar(cur_z, 80, 24, bar_buf, sizeof(bar_buf));
            printf("\r\033[K  %s Z=%-3d  [Pair %d/3] %s",
                   bar_buf, cur_z, prof.double_tap_count + 1,
                   phase3_sub == 1 ? ">>> REBOUND CLICK NOW <<<" : "(Perform natural double-click...)");
            fflush(stdout);
        }
    }

    if (!g_cal_running) goto cleanup_exit;

    prof.double_tap1_avg_peak = (int)(d_t1_sum / (uint64_t)prof.double_tap_count);
    prof.double_tap2_avg_peak = (int)(d_t2_sum / (uint64_t)prof.double_tap_count);
    prof.double_tap_avg_gap_ms = (uint32_t)(d_gap_sum / (uint64_t)prof.double_tap_count);
    printf("  ✓ Step 3 Complete: 3 double-clicks recorded (Avg Rebound=%d, Avg Gap=%ums)\n\n",
           prof.double_tap2_avg_peak, prof.double_tap_avg_gap_ms);
    fflush(stdout);

    int rep3 = cal_prompt_choice("Keep these double-clicks [Enter] or [r]etry Step 3? [Y/r]: ", "yr", 'y');
    if (rep3 == 'r') goto phase3_retry;

    cal_wait_for_user("\nPress [Enter] to continue to Step 4...");

    /* =========================================================================
     * Step 4/4: Steering Shear Profile (3.0 seconds active navigation)
     * ========================================================================= */
phase4_retry:
    if (!g_cal_running) goto cleanup_exit;

    prof.steer_max_z = 0;
    prof.steer_max_motion = 0;

    printf("\n------------------------------------------------------------------------------\n");
    printf("Step 4 of 4: Steering Shear Profile\n");
    printf("------------------------------------------------------------------------------\n");
    printf("INSTRUCTIONS:\n");
    printf("  1. In this step, you will steer the trackstick in small continuous circles\n");
    printf("     for 3 seconds, as if you are moving the mouse pointer around the screen.\n");
    printf("  2. This teaches the daemon the difference between intentional cursor steering\n");
    printf("     and tapping strikes, preventing cursor navigation from misfiring clicks.\n");
    printf("  3. A 3-second countdown will prepare you before measurement begins.\n\n");
    fflush(stdout);

    cal_wait_for_user("--> Place finger on the nub and press [Enter] when ready to steer: ");

    printf("Get ready... "); fflush(stdout);
    if (isatty(STDIN_FILENO)) {
        for (int c = 3; c >= 1; c--) {
            printf("%d... ", c); fflush(stdout);
            usleep(600000);
        }
    }
    printf("STEER IN CIRCLES NOW!\n"); fflush(stdout);

    uint64_t p4_start_ms = cal_get_time_ms();
    int cur_shear = 0;
    cur_z = 0;

    while (g_cal_running) {
        uint64_t now_ms = cal_get_time_ms();
        if (now_ms - p4_start_ms >= 3000) {
            break;
        }

        struct epoll_event ep_ev[1];
        int nfds = epoll_wait(epfd, ep_ev, 1, 50);

        if (nfds > 0) {
            ssize_t bytes = read(stick_fd, raw_events, sizeof(raw_events));
            if (bytes > 0) {
                size_t count = (size_t)bytes / sizeof(struct input_event);
                for (size_t i = 0; i < count; i++) {
                    if (raw_events[i].type == EV_ABS && raw_events[i].code == ABS_PRESSURE) {
                        cur_z = raw_events[i].value;
                        if (cur_z > prof.steer_max_z) {
                            prof.steer_max_z = cur_z;
                        }
                    } else if (raw_events[i].type == EV_REL) {
                        int val = abs(raw_events[i].value);
                        cur_shear = val;
                        if (val > prof.steer_max_motion) {
                            prof.steer_max_motion = val;
                        }
                    }
                }
            }
        }

        if (now_ms - last_render_ms >= 50) {
            last_render_ms = now_ms;
            uint64_t elapsed = now_ms - p4_start_ms;
            int pct = (int)((elapsed * 100) / 3000);
            if (pct > 100) pct = 100;
            alps_calibrate_format_bar((int)elapsed, 3000, 24, bar_buf, sizeof(bar_buf));
            printf("\r\033[K  %s %3d%%  (Z=%d, Shear=%d, Max Steer Z: %d, Max Shear: %d)",
                   bar_buf, pct, cur_z, cur_shear, prof.steer_max_z, prof.steer_max_motion);
            fflush(stdout);
        }
    }

    if (!g_cal_running) goto cleanup_exit;

    printf("\r\033[K  ✓ Step 4 Complete: Max Vertical Z=%d, Max Lateral Shear=%d counts\n\n",
           prof.steer_max_z, prof.steer_max_motion);
    fflush(stdout);

    if (prof.steer_max_motion < 6) {
        printf("Notice: Very little or no steering shear was observed (Max shear=%d counts).\n",
               prof.steer_max_motion);
        int rep4 = cal_prompt_choice("Would you like to retry Step 4? [Y/n]: ", "yn", 'y');
        if (rep4 == 'y') goto phase4_retry;
    }

    /* Compute tailored optimal recommendations */
    alps_calibrate_compute_guided_recommendations(&prof);

    printf("==============================================================================\n");
    printf("  Calibration Results & Optimal Parameter Derivation\n");
    printf("==============================================================================\n");
    printf("Observed Physical Characteristics:\n");
    printf("  - Resting Baseline Noise Floor:   Z = %d (Peak Noise: %d)\n",
           prof.noise_floor_z, prof.noise_floor_max_z);
    printf("  - Single Tap Peak Force:          Avg = %d counts (Min: %d, Max: %d)\n",
           prof.single_tap_avg_peak, prof.single_tap_min_peak, prof.single_tap_max_peak);
    printf("  - Contact Dwell Duration:         Avg = %u ms\n", prof.single_tap_avg_dur_ms);
    printf("  - Double-Click Rebound Force:     Avg = %d counts (First tap: %d)\n",
           prof.double_tap2_avg_peak, prof.double_tap1_avg_peak);
    printf("  - Double-Click Cadence Gap:       Avg = %u ms\n", prof.double_tap_avg_gap_ms);
    printf("  - Navigation Steering Force:      Max Z = %d, Max Shear = %d counts\n\n",
           prof.steer_max_z, prof.steer_max_motion);

    printf("Recommended Configuration for %s:\n", target_config_path);
    printf("  [tap]\n");
    printf("  tap1_pressure = %d\n", prof.rec_tap1_pressure);
    printf("  tap2_pressure = %d\n", prof.rec_tap2_pressure);
    printf("  release_pressure = %d\n", prof.rec_release_pressure);
    printf("  tap_duration_ms = %u\n", prof.rec_tap_duration_ms);
    printf("  tap_motion = %d\n", prof.rec_tap_motion);
    printf("  double_tap_window_ms = %u\n\n", prof.rec_double_window_ms);

    printf("------------------------------------------------------------------------------\n");
    printf("What would you like to do?\n");
    printf("  [1] Save and apply these settings to %s (Recommended)\n", target_config_path);
    printf("  [2] Re-run the full 4-phase calibration from the beginning\n");
    printf("  [3] Reset configuration file to factory defaults\n");
    printf("  [4] Exit without saving\n");
    printf("------------------------------------------------------------------------------\n");

    int final_choice = cal_prompt_choice("Choice [1-4] (default: 1): ", "1234", '1');

    if (final_choice == '1') {
        int res = alps_calibrate_save_config(target_config_path, &prof);
        if (res == 0) {
            printf("\n✓ Successfully updated %s!\n", target_config_path);
            alps_service_offer_restart("alps-stick", 0);
        } else if (res == -2) {
            printf("\nNotice: Permission denied writing directly to %s.\n", target_config_path);
            const char *fallback_path = "/tmp/alps-stick.conf";
            int fb_res = alps_calibrate_save_config(fallback_path, &prof);
            if (fb_res == 0) {
                printf("Calibrated settings successfully saved to %s.\n\n", fallback_path);
                printf("To apply permanently to your system, run:\n");
                printf("  sudo cp %s %s\n", fallback_path, target_config_path);
                printf("  sudo systemctl restart alps-stick\n\n");
            } else {
                printf("Failed to write to %s as well (code %d).\n\n", fallback_path, fb_res);
            }
        } else {
            printf("\nError: Failed to save configuration to %s (code %d).\n\n", target_config_path, res);
        }
    } else if (final_choice == '2') {
        goto restart_wizard;
    } else if (final_choice == '3') {
        int rst_res = alps_config_reset_file(target_config_path);
        if (rst_res == 0) {
            printf("\n✓ Successfully restored %s to factory defaults!\n", target_config_path);
            alps_service_offer_restart("alps-stick", 0);
        } else if (rst_res == -2) {
            printf("\nNotice: Permission denied writing directly to %s.\n", target_config_path);
            const char *fallback_path = "/tmp/alps-stick.conf";
            int fb_res = alps_config_reset_file(fallback_path);
            if (fb_res == 0) {
                printf("Default configuration written to %s.\n\n", fallback_path);
                printf("To apply permanently to your system, run:\n");
                printf("  sudo cp %s %s\n", fallback_path, target_config_path);
                printf("  sudo systemctl restart alps-stick\n\n");
            }
        }
    } else {
        printf("\nExited without modifying configuration.\n\n");
    }

cleanup_exit:
    close(epfd);
    close(stick_fd);

    if (!g_cal_running) {
        printf("\n\nCalibration cancelled by user.\n");
    }

    return 0;
}

int alps_calibrate_run_hud(const alps_config_t *cfg) {
    (void)cfg;
    char dev_path[512] = {0};
    char dev_name[256] = {0};

    int stick_fd = alps_device_find_stick(dev_path, sizeof(dev_path), dev_name, sizeof(dev_name));
    if (stick_fd < 0) {
        fprintf(stderr, "\nError: ALPS DualPoint Stick not found in /dev/input/event*!\n");
        return 1;
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = cal_sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        close(stick_fd);
        return 1;
    }

    struct epoll_event ev_sub;
    memset(&ev_sub, 0, sizeof(ev_sub));
    ev_sub.events = EPOLLIN;
    ev_sub.data.fd = stick_fd;
    epoll_ctl(epfd, EPOLL_CTL_ADD, stick_fd, &ev_sub);

    if (isatty(STDIN_FILENO)) {
        struct epoll_event ev_stdin;
        memset(&ev_stdin, 0, sizeof(ev_stdin));
        ev_stdin.events = EPOLLIN;
        ev_stdin.data.fd = STDIN_FILENO;
        epoll_ctl(epfd, EPOLL_CTL_ADD, STDIN_FILENO, &ev_stdin);
    }

    alps_calibrate_tracker_t tracker;
    alps_calibrate_tracker_init(&tracker);

    printf("\n==============================================================================\n");
    printf("  ALPS Trackstick Live Diagnostic Telemetry HUD\n");
    printf("==============================================================================\n");
    printf("Connected Device: %s (%s)\n", dev_name, dev_path);
    printf("Press Ctrl+C or 'q' [Enter] to exit HUD.\n");
    printf("------------------------------------------------------------------------------\n");
    fflush(stdout);

    struct input_event raw_events[32];
    char bar_buf[64];
    uint64_t last_render_ms = 0;
    int tap_flash = 0;
    g_cal_running = 1;

    while (g_cal_running) {
        struct epoll_event ep_events[2];
        int nfds = epoll_wait(epfd, ep_events, 2, 50);
        uint64_t now_ms = cal_get_time_ms();

        for (int i = 0; i < nfds; i++) {
            if (ep_events[i].data.fd == STDIN_FILENO) {
                char ch = 0;
                if (read(STDIN_FILENO, &ch, 1) > 0) {
                    if (ch == 'q' || ch == 'Q') {
                        g_cal_running = 0;
                        break;
                    }
                }
            } else if (ep_events[i].data.fd == stick_fd) {
                ssize_t bytes = read(stick_fd, raw_events, sizeof(raw_events));
                if (bytes > 0) {
                    size_t count = (size_t)bytes / sizeof(struct input_event);
                    for (size_t j = 0; j < count; j++) {
                        alps_event_t norm_ev;
                        int valid = 0;

                        if (raw_events[j].type == EV_ABS && raw_events[j].code == ABS_PRESSURE) {
                            norm_ev.type = ALPS_EV_PRESSURE;
                            norm_ev.value = raw_events[j].value;
                            valid = 1;
                        } else if (raw_events[j].type == EV_REL) {
                            if (raw_events[j].code == REL_X) {
                                norm_ev.type = ALPS_EV_REL_X;
                                norm_ev.value = raw_events[j].value;
                                valid = 1;
                            } else if (raw_events[j].code == REL_Y) {
                                norm_ev.type = ALPS_EV_REL_Y;
                                norm_ev.value = raw_events[j].value;
                                valid = 1;
                            }
                        }

                        if (valid) {
                            if (alps_calibrate_tracker_feed_event(&tracker, &norm_ev, now_ms)) {
                                tap_flash = 10;
                            }
                        }
                    }
                }
            }
        }

        if (now_ms - last_render_ms >= 50) {
            last_render_ms = now_ms;
            alps_calibrate_format_bar(tracker.current_z, 80, 26, bar_buf, sizeof(bar_buf));

            const char *status_str = "Resting";
            if (tap_flash > 0) {
                status_str = ">>> TAP REGISTERED! <<<";
                tap_flash--;
            } else if (tracker.active_press_start_ms > 0) {
                if (tracker.active_motion_counts >= 6) {
                    status_str = "Steering Shear (Navigating)";
                } else {
                    status_str = "Contact Press Active";
                }
            }

            printf("\r\033[K%s  Z=%-3d (Peak=%-2d)  dX=%-+3d dY=%-+3d  [%s]  Taps: %d",
                   bar_buf,
                   tracker.current_z,
                   tracker.active_press_start_ms > 0 ? tracker.active_peak_z : (tracker.tap_count > 0 ? tracker.max_tap_z : 0),
                   tracker.current_dx,
                   tracker.current_dy,
                   status_str,
                   tracker.tap_count);
            fflush(stdout);
        }
    }

    close(epfd);
    close(stick_fd);
    printf("\n\nExiting Diagnostic HUD.\n");
    return 0;
}

int alps_interactive_menu_run(const alps_config_t *cfg) {
    g_cal_running = 1;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = cal_sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    while (g_cal_running) {
        printf("\n");
        printf("==============================================================================\n");
        printf("  ALPS DualPoint Trackstick — Interactive Control Center (v%s)\n", ALPS_DAEMON_VERSION);
        printf("==============================================================================\n");

        char dev_path[256] = {0};
        char dev_name[256] = {0};
        int stick_fd = alps_device_find_stick(dev_path, sizeof(dev_path), dev_name, sizeof(dev_name));
        if (stick_fd >= 0) {
            close(stick_fd);
            printf("  Hardware Device:    ● CONNECTED (%s on %s)\n", dev_name, dev_path);
        } else {
            printf("  Hardware Device:    ○ NOT DETECTED (/dev/input/event*)\n");
        }

        int svc_active = alps_systemd_is_active("alps-stick");
        if (svc_active == 1) {
            printf("  Daemon Service:     ● ACTIVE (alps-stick.service running)\n");
        } else if (svc_active == 0) {
            printf("  Daemon Service:     ○ INACTIVE (alps-stick.service stopped)\n");
        } else {
            printf("  Daemon Service:     ? UNMANAGED (systemd unavailable)\n");
        }

        const char *conf_path = (cfg && cfg->config_path[0] != '\0') ? cfg->config_path : ALPS_DEFAULT_CONFIG_PATH;
        printf("  Configuration File: %s\n", conf_path);
        printf("------------------------------------------------------------------------------\n");
        printf("Actions:\n");
        printf("  [1] Run Guided 4-Phase Physical Calibration Wizard\n");
        printf("  [2] Live Transducer Force & Motion Telemetry (Diagnostic HUD)\n");
        printf("  [3] View Current Active Configuration File\n");
        printf("  [4] Reset Configuration File to Factory Defaults\n");
        printf("  [5] Restart Background Service (alps-stick.service)\n");
        printf("  [6] View Recent Daemon Service Logs (journalctl)\n");
        printf("  [q] Quit Control Center\n");
        printf("------------------------------------------------------------------------------\n");
        printf("Select an action [1-6, q]: ");
        fflush(stdout);

        char line[64];
        if (!fgets(line, sizeof(line), stdin)) {
            break;
        }

        char *p = line;
        while (isspace((unsigned char)*p)) p++;

        if (*p == '1') {
            alps_calibrate_run(cfg);
        } else if (*p == '2') {
            alps_calibrate_run_hud(cfg);
        } else if (*p == '3') {
            printf("\n--- Current Configuration (%s) ---\n", conf_path);
            FILE *f = fopen(conf_path, "r");
            if (f) {
                char cline[512];
                while (fgets(cline, sizeof(cline), f)) {
                    fputs(cline, stdout);
                }
                fclose(f);
            } else {
                printf("Notice: Configuration file does not exist yet. Built-in defaults are active.\n");
            }
            cal_wait_for_user("\nPress [Enter] to return to menu...");
        } else if (*p == '4') {
            int rc = alps_config_reset_file(conf_path);
            if (rc == 0) {
                printf("\n✓ Successfully restored '%s' to factory defaults.\n", conf_path);
                alps_service_offer_restart("alps-stick", 0);
            } else if (rc == -2) {
                printf("\nPermission denied writing to '%s'. Run with sudo.\n", conf_path);
            } else {
                printf("\nFailed to reset configuration file.\n");
            }
            cal_wait_for_user("Press [Enter] to return to menu...");
        } else if (*p == '5') {
            printf("\nRestarting alps-stick.service...\n");
            int rc = alps_systemd_restart("alps-stick");
            if (rc == 0) {
                printf("✓ Successfully restarted alps-stick.service!\n");
            } else {
                printf("Notice: Could not restart service. Run manually: sudo systemctl restart alps-stick\n");
            }
            cal_wait_for_user("Press [Enter] to return to menu...");
        } else if (*p == '6') {
            printf("\n--- Recent Logs (journalctl -u alps-stick.service -n 25) ---\n");
            pid_t pid = fork();
            if (pid == 0) {
                execlp("journalctl", "journalctl", "-u", "alps-stick.service", "-n", "25", "--no-pager", NULL);
                _exit(1);
            } else if (pid > 0) {
                int status = 0;
                waitpid(pid, &status, 0);
            }
            cal_wait_for_user("\nPress [Enter] to return to menu...");
        } else if (*p == 'q' || *p == 'Q') {
            break;
        }
    }

    printf("\nExiting Control Center.\n");
    return 0;
}
