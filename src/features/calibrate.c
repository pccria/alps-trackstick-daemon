/**
 * @file calibrate.c
 * @brief Implementation of guided 4-phase physical calibration and force telemetry.
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

static char *cal_trim_whitespace(char *str) {
    if (!str) return NULL;
    while (isspace((unsigned char)*str)) str++;
    if (*str == '\0') return str;
    char *end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    end[1] = '\0';
    return str;
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
        /* Destination config does not exist: generate complete standard INI template */
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

    /* Existing configuration: update [tap] section while preserving comments and other settings */
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

        /* Section header detection */
        if (*p == '[') {
            char *end_b = strchr(p, ']');
            if (end_b) {
                if (in_tap_sec) {
                    /* Flush missing keys before exiting [tap] */
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

    printf("\n");
    printf("==============================================================================\n");
    printf("  ALPS Trackstick Guided 4-Phase Physical Calibration Wizard (v%s)\n", ALPS_DAEMON_VERSION);
    printf("==============================================================================\n");
    printf("Connected Hardware: %s (%s)\n", dev_name, dev_path);
    printf("Target Config File: %s\n\n", target_config_path);
    printf("This guided calibration profiles your specific strain-gauge transducer and\n");
    printf("tailors tap, rebound, and release parameters to your natural tactile cadence.\n");
    printf("Press Ctrl+C at any time to exit without modifying your configuration.\n\n");
    fflush(stdout);

    alps_guided_profile_t prof;
    alps_calibrate_profile_init(&prof);

    struct input_event raw_events[32];
    char bar_buf[64];

    /* =========================================================================
     * Phase 1/4: Resting Baseline & Noise Floor (2.0 seconds hands-off)
     * ========================================================================= */
    printf("------------------------------------------------------------------------------\n");
    printf("Phase 1/4: Resting Baseline & Noise Floor\n");
    printf("Please DO NOT touch the trackstick for 2 seconds while the baseline settles...\n");
    printf("------------------------------------------------------------------------------\n");
    fflush(stdout);

    uint64_t p1_start_ms = cal_get_time_ms();
    uint64_t last_render_ms = 0;
    uint64_t noise_z_sum = 0;
    uint32_t noise_samples = 0;
    int cur_z = 0;

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

    if (!g_cal_running) {
        close(epfd);
        close(stick_fd);
        printf("\n\nCalibration cancelled by user.\n");
        return 0;
    }

    prof.noise_floor_z = noise_samples > 0 ? (int)(noise_z_sum / noise_samples) : 0;
    printf("\r\033[K  ✓ Baseline established: Resting Z=%d (Transient Noise Floor Max=%d)\n\n",
           prof.noise_floor_z, prof.noise_floor_max_z);
    fflush(stdout);

    /* =========================================================================
     * Phase 2/4: Natural Single Taps (5 repetitions)
     * ========================================================================= */
    printf("------------------------------------------------------------------------------\n");
    printf("Phase 2/4: Natural Single Taps\n");
    printf("Tap the trackstick naturally 5 times with your index finger (like clicking).\n");
    printf("------------------------------------------------------------------------------\n");
    fflush(stdout);

    uint64_t contact_start_ms = 0;
    int active_peak_z = 0;
    int active_shear = 0;
    uint64_t single_peak_sum = 0;
    uint64_t single_dur_sum = 0;
    cur_z = 0;

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
            printf("\r\033[K  %s Z=%-3d  [Tap %d/5] (Waiting for tap strike...)",
                   bar_buf, cur_z, prof.single_tap_count + 1);
            fflush(stdout);
        }
    }

    if (!g_cal_running) {
        close(epfd);
        close(stick_fd);
        printf("\n\nCalibration cancelled by user.\n");
        return 0;
    }

    prof.single_tap_avg_peak = (int)(single_peak_sum / (uint64_t)prof.single_tap_count);
    prof.single_tap_avg_dur_ms = (uint32_t)(single_dur_sum / (uint64_t)prof.single_tap_count);
    printf("  ✓ Single tap profile: Avg Peak=%d (Min: %d, Max: %d), Avg Dwell=%ums\n\n",
           prof.single_tap_avg_peak, prof.single_tap_min_peak, prof.single_tap_max_peak,
           prof.single_tap_avg_dur_ms);
    fflush(stdout);

    /* =========================================================================
     * Phase 3/4: Double-Click Cadence & Rebound (3 repetitions)
     * ========================================================================= */
    printf("------------------------------------------------------------------------------\n");
    printf("Phase 3/4: Double-Click Cadence & Rebound\n");
    printf("Perform 3 natural double-clicks (tap-tap) at your normal double-click rhythm.\n");
    printf("------------------------------------------------------------------------------\n");
    fflush(stdout);

    int phase3_sub = 0; /* 0: waiting for tap 1, 1: waiting for tap 2 */
    int t1_peak = 0;
    uint64_t t1_end_ms = 0;
    uint64_t d_t1_sum = 0;
    uint64_t d_t2_sum = 0;
    uint64_t d_gap_sum = 0;
    contact_start_ms = 0;
    active_peak_z = 0;
    cur_z = 0;

    while (g_cal_running && prof.double_tap_count < 3) {
        struct epoll_event ep_ev[1];
        int nfds = epoll_wait(epfd, ep_ev, 1, 50);
        uint64_t now_ms = cal_get_time_ms();

        /* Reset pair if inter-tap pause exceeds 500ms */
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

    if (!g_cal_running) {
        close(epfd);
        close(stick_fd);
        printf("\n\nCalibration cancelled by user.\n");
        return 0;
    }

    prof.double_tap1_avg_peak = (int)(d_t1_sum / (uint64_t)prof.double_tap_count);
    prof.double_tap2_avg_peak = (int)(d_t2_sum / (uint64_t)prof.double_tap_count);
    prof.double_tap_avg_gap_ms = (uint32_t)(d_gap_sum / (uint64_t)prof.double_tap_count);
    printf("  ✓ Double-click cadence: Avg Rebound=%d (Tap 1: %d), Avg Gap=%ums\n\n",
           prof.double_tap2_avg_peak, prof.double_tap1_avg_peak, prof.double_tap_avg_gap_ms);
    fflush(stdout);

    /* =========================================================================
     * Phase 4/4: Steering Shear Profile (3.0 seconds active navigation)
     * ========================================================================= */
    printf("------------------------------------------------------------------------------\n");
    printf("Phase 4/4: Steering Shear Profile\n");
    printf("Push and steer the trackstick in circles for 3 seconds as if navigating.\n");
    printf("------------------------------------------------------------------------------\n");
    fflush(stdout);

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

    close(epfd);
    close(stick_fd);

    if (!g_cal_running) {
        printf("\n\nCalibration cancelled by user.\n");
        return 0;
    }

    printf("\r\033[K  ✓ Steering profile: Max Vertical Z=%d, Max Lateral Shear=%d counts\n\n",
           prof.steer_max_z, prof.steer_max_motion);
    fflush(stdout);

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

    int action = 0; /* 0: do nothing, 1: save calibrated, 2: reset defaults */
    if (isatty(STDIN_FILENO)) {
        printf("Apply settings to %s? [Y]es (save) / [n]o / [r]eset to defaults: ", target_config_path);
        fflush(stdout);
        char ans[64];
        if (fgets(ans, sizeof(ans), stdin)) {
            char *p = cal_trim_whitespace(ans);
            if (*p == '\0' || *p == 'y' || *p == 'Y') {
                action = 1;
            } else if (*p == 'r' || *p == 'R') {
                action = 2;
            }
        }
    }

    if (action == 1) {
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
    } else if (action == 2) {
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
            } else {
                printf("Failed to write to %s as well (code %d).\n\n", fallback_path, fb_res);
            }
        } else {
            printf("\nError: Failed to reset configuration at %s (code %d).\n\n", target_config_path, rst_res);
        }
    } else if (isatty(STDIN_FILENO)) {
        printf("\nSettings not saved. You may manually copy the [tap] block above into %s.\n\n",
               target_config_path);
    }

    return 0;
}
