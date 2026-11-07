/**
 * @file calibrate.c
 * @brief Implementation of interactive terminal calibration and force telemetry.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "features/calibrate.h"
#include "io/device.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

void alps_calibrate_tracker_init(alps_calibrate_tracker_t *tracker) {
    if (!tracker) return;
    memset(tracker, 0, sizeof(*tracker));
    tracker->min_tap_z = 255;
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

    alps_calibrate_tracker_t tracker;
    alps_calibrate_tracker_init(&tracker);

    printf("\n");
    printf("==============================================================================\n");
    printf("  ALPS Trackstick Interactive Transducer Calibration (v%s)\n", ALPS_DAEMON_VERSION);
    printf("==============================================================================\n");
    printf("Connected Device: %s (%s)\n", dev_name, dev_path);
    printf("Current Config:   TAP1_Z=%d, TAP2_Z=%d, RELEASE_Z=%d\n\n",
           cfg ? cfg->tap.tap1_pressure : ALPS_DEFAULT_TAP1_PRESSURE,
           cfg ? cfg->tap.tap2_rebound_pressure : ALPS_DEFAULT_TAP2_REBOUND_PRESSURE,
           cfg ? cfg->tap.release_pressure : ALPS_DEFAULT_RELEASE_PRESSURE);
    printf("Instructions:\n");
    printf("  1. Tap the physical trackstick lightly 5 to 10 times at your natural force.\n");
    printf("  2. Watch the live force meter and contact duration in real time.\n");
    printf("  3. Press Ctrl+C when finished to generate your customized config snippet.\n\n");
    printf("------------------------------------------------------------------------------\n");
    fflush(stdout);

    struct input_event raw_events[32];
    char bar_buf[64];
    uint64_t last_render_ms = 0;
    int tap_flash = 0;

    while (g_cal_running) {
        struct epoll_event ep_events[1];
        int nfds = epoll_wait(epfd, ep_events, 1, 50);

        uint64_t now_ms = cal_get_time_ms();

        if (nfds > 0) {
            ssize_t bytes = read(stick_fd, raw_events, sizeof(raw_events));
            if (bytes > 0) {
                size_t count = (size_t)bytes / sizeof(struct input_event);
                for (size_t i = 0; i < count; i++) {
                    alps_event_t norm_ev;
                    int valid = 0;

                    if (raw_events[i].type == EV_ABS && raw_events[i].code == ABS_PRESSURE) {
                        norm_ev.type = ALPS_EV_PRESSURE;
                        norm_ev.value = raw_events[i].value;
                        valid = 1;
                    } else if (raw_events[i].type == EV_REL) {
                        if (raw_events[i].code == REL_X) {
                            norm_ev.type = ALPS_EV_REL_X;
                            norm_ev.value = raw_events[i].value;
                            valid = 1;
                        } else if (raw_events[i].code == REL_Y) {
                            norm_ev.type = ALPS_EV_REL_Y;
                            norm_ev.value = raw_events[i].value;
                            valid = 1;
                        }
                    }

                    if (valid) {
                        if (alps_calibrate_tracker_feed_event(&tracker, &norm_ev, now_ms)) {
                            tap_flash = 10; /* Flash notification for 10 frames */
                        }
                    }
                }
            }
        }

        /* Render terminal dashboard at ~20 Hz */
        if (now_ms - last_render_ms >= 50) {
            last_render_ms = now_ms;
            alps_calibrate_format_bar(tracker.current_z, 80, 28, bar_buf, sizeof(bar_buf));

            int avg_peak = tracker.tap_count > 0 ? (int)(tracker.tap_peak_sum / (uint64_t)tracker.tap_count) : 0;
            int avg_dur = tracker.tap_count > 0 ? (int)(tracker.tap_duration_sum / (uint64_t)tracker.tap_count) : 0;

            const char *status_str = "Resting";
            if (tap_flash > 0) {
                status_str = ">>> TAP REGISTERED! <<<";
                tap_flash--;
            } else if (tracker.active_press_start_ms > 0) {
                if (tracker.active_motion_counts >= 6) {
                    status_str = "Steering Shear (Drag)";
                } else {
                    status_str = "Contact Press Active";
                }
            }

            printf("\r\033[K%s  Z=%-3d  (Peak=%-2d)  dX=%-+3d dY=%-+3d  [%s]  Taps: %d (AvgPeak:%d Dur:%dms)",
                   bar_buf,
                   tracker.current_z,
                   tracker.active_press_start_ms > 0 ? tracker.active_peak_z : (tracker.tap_count > 0 ? tracker.max_tap_z : 0),
                   tracker.current_dx,
                   tracker.current_dy,
                   status_str,
                   tracker.tap_count,
                   avg_peak,
                   avg_dur);
            fflush(stdout);
        }
    }

    close(epfd);
    close(stick_fd);

    printf("\n\n------------------------------------------------------------------------------\n");
    printf("Calibration Session Complete (%d taps recorded)\n", tracker.tap_count);
    printf("------------------------------------------------------------------------------\n");

    int rec_tap1, rec_tap2, rec_release;
    alps_calibrate_compute_recommendation(&tracker, &rec_tap1, &rec_tap2, &rec_release);

    if (tracker.tap_count >= 2) {
        int avg_peak = (int)(tracker.tap_peak_sum / (uint64_t)tracker.tap_count);
        int avg_dur = (int)(tracker.tap_duration_sum / (uint64_t)tracker.tap_count);

        printf("Observed Physical Metrics:\n");
        printf("  - Average Peak Tap Force: %d counts (Min: %d, Max: %d)\n",
               avg_peak, tracker.min_tap_z, tracker.max_tap_z);
        printf("  - Average Contact Duration: %d ms\n\n", avg_dur);
    } else {
        printf("Note: Fewer than 2 taps recorded; displaying calibrated default values.\n\n");
    }

    printf("# Tailored Settings for /etc/alps-stick.conf:\n");
    printf("[tap]\n");
    printf("tap1_pressure = %d\n", rec_tap1);
    printf("tap2_pressure = %d\n", rec_tap2);
    printf("release_pressure = %d\n\n", rec_release);

    return 0;
}
