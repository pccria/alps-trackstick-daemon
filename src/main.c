/**
 * @file main.c
 * @brief Daemon entry point, `epoll` event loop, and feature pipeline orchestrator.
 *
 * Responsibilities:
 * 1. Parse command-line configuration (`alps_config_parse_args`).
 * 2. Maintain a persistent virtual `/dev/uinput` emitter (`alps_emitter_create`)
 *    across hardware suspend/resume cycles.
 * 3. Monitor `/dev/input` for ALPS DualPoint Stick appearance via `inotify`.
 * 4. Multiplex raw kernel input events via `epoll_wait` with dynamic timeouts
 *    queried from active feature modules.
 * 5. Normalize raw `struct input_event` packets into `alps_event_t` structs
 *    and route them through the feature pipeline (`tap_gesture` and `scroll_wheel`).
 * 6. Guarantee clean virtual button release (`alps_tap_gesture_reset`) on
 *    `SIGINT`/`SIGTERM` or device disconnect.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <sys/epoll.h>
#include <linux/input.h>

#include "core/config.h"
#include "core/event.h"
#include "core/action.h"
#include "features/tap_gesture.h"
#include "features/scroll_wheel.h"
#include "io/device.h"
#include "io/emitter.h"

static volatile sig_atomic_t g_running = 1;

static void sig_handler(int sig) {
    (void)sig;
    g_running = 0;
}

/**
 * @brief Installs `SIGINT` and `SIGTERM` handlers without `SA_RESTART` so
 *        `epoll_wait` unblocks immediately on shutdown signals.
 */
static int setup_signals(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (sigaction(SIGINT, &sa, NULL) < 0) return -1;
    if (sigaction(SIGTERM, &sa, NULL) < 0) return -1;
    return 0;
}

/**
 * @brief Returns the current `CLOCK_MONOTONIC` timestamp in milliseconds.
 */
static uint64_t get_time_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0) {
        return 0;
    }
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
}

/**
 * @brief Safely detaches a disconnected or suspended trackstick.
 *
 * Removes the device from the epoll set, closes its file descriptor, resets
 * the gesture and scroll state machines, and dispatches any necessary button-up
 * actions so desktop environments do not suffer from stuck virtual mouse buttons.
 */
static void handle_device_disconnect(int *stick_fd, int epfd, int ufd,
                                     alps_tap_gesture_t *tap_ctx,
                                     alps_scroll_wheel_t *scroll_ctx,
                                     int verbose) {
    if (!stick_fd || *stick_fd < 0) return;

    if (epfd >= 0) {
        epoll_ctl(epfd, EPOLL_CTL_DEL, *stick_fd, NULL);
    }
    close(*stick_fd);
    *stick_fd = -1;

    /* Guarantee virtual buttons and accumulators are released while disconnected */
    alps_action_t rst_act = alps_tap_gesture_reset(tap_ctx);
    alps_emitter_dispatch(ufd, &rst_act);
    alps_scroll_wheel_reset(scroll_ctx);

    if (verbose) {
        printf("Hardware disconnected / suspended. Awaiting hardware re-enumeration...\n");
        fflush(stdout);
    }
}

/**
 * @brief Attempts to discover and connect to the ALPS DualPoint Stick.
 *
 * Scans `/dev/input/event*` for the stick node. If found, opens it, registers
 * it with epoll, and optionally re-applies libinput pointer speed.
 *
 * @return 0 on successful connection or already connected, -1 if hardware not found.
 */
static int try_reconnect_device(int *stick_fd, int epfd, const alps_config_t *cfg,
                                char *dev_path, size_t path_max,
                                char *dev_name, size_t name_max) {
    if (!stick_fd || *stick_fd >= 0) return 0;

    int fd = alps_device_find_stick(dev_path, path_max, dev_name, name_max);
    if (fd < 0) return -1;

    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN;
    ev.data.fd = fd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) < 0) {
        perror("epoll_ctl add device failed");
        close(fd);
        return -1;
    }

    *stick_fd = fd;

    if (cfg->pointer_speed >= -1.0f && cfg->pointer_speed <= 1.0f) {
        alps_device_apply_pointer_speed(dev_name, cfg->pointer_speed);
    }

    if (cfg->verbose) {
        printf("Connected: %s (%s)\n", dev_name, dev_path);
        fflush(stdout);
    }
    return 0;
}

int main(int argc, char *argv[]) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    int parse_rc = alps_config_parse_args(&cfg, argc, argv);
    if (parse_rc > 0) {
        return 0; /* --help or --version handled */
    }
    if (parse_rc < 0) {
        return 1;
    }

    if (setup_signals() < 0) {
        perror("Failed to configure signals");
        return 1;
    }

    int ufd = alps_emitter_create();
    if (ufd < 0) {
        fprintf(stderr, "Error: Failed to initialize uinput device!\n");
        return 1;
    }

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        perror("epoll_create1 failed");
        alps_emitter_destroy(ufd);
        return 1;
    }

    int monitor_fd = alps_device_monitor_create();
    if (monitor_fd >= 0) {
        struct epoll_event ev_mon;
        memset(&ev_mon, 0, sizeof(ev_mon));
        ev_mon.events = EPOLLIN;
        ev_mon.data.fd = monitor_fd;
        if (epoll_ctl(epfd, EPOLL_CTL_ADD, monitor_fd, &ev_mon) < 0) {
            perror("Warning: Failed to add inotify monitor to epoll");
            alps_device_monitor_destroy(monitor_fd);
            monitor_fd = -1;
        }
    }

    char dev_path[512] = {0};
    char dev_name[256] = {0};
    int stick_fd = -1;
    try_reconnect_device(&stick_fd, epfd, &cfg, dev_path, sizeof(dev_path), dev_name, sizeof(dev_name));

    if (cfg.verbose) {
        printf("ALPS Native C Daemon v%s (Modular 2D Kinetic Transducer Engine)\n",
               ALPS_DAEMON_VERSION);
        if (stick_fd >= 0) {
            printf("Connected: %s (%s)\n", dev_name, dev_path);
        } else {
            printf("Hardware not present at launch. Awaiting device appearance...\n");
        }
        if (cfg.pointer_speed >= -1.0f && cfg.pointer_speed <= 1.0f) {
            printf("Pointer Speed Override: %.2f (Forced Adaptive Profile)\n", cfg.pointer_speed);
        } else {
            printf("Pointer Ballistics: Native (KDE/System acceleration profile untouched)\n");
        }
        printf("Tap Calibration: TAP1_Z=%d, TAP2_Z=%d, RELEASE_Z=%d, TAP1_MAX_MOTION=%d, "
               "DRAG_MOTION=%d, DRAG_GRACE=%ums, ANTI_DRIFT=%s\n",
               cfg.tap.tap1_pressure, cfg.tap.tap2_rebound_pressure, cfg.tap.release_pressure,
               cfg.tap.max_tap_motion, cfg.tap.drag_start_motion, cfg.tap.drag_release_grace_ms,
               cfg.tap.anti_drift_enabled ? "ON" : "OFF");
        if (cfg.scroll.enabled) {
            printf("2D Kinetic Scroll: Enabled (Middle-Button Hold, Hi-Res Multiplier=%d, Axis-Lock=%s, Counter-Drift=%s)\n",
                   cfg.scroll.hires_multiplier,
                   cfg.scroll.axis_lock_enabled ? "ON" : "OFF",
                   cfg.scroll.counter_drift_enabled ? "ON" : "OFF");
        } else {
            printf("2D Kinetic Scroll: Disabled\n");
        }
        fflush(stdout);
    }

    alps_tap_gesture_t tap_ctx;
    alps_tap_gesture_init(&tap_ctx);

    alps_scroll_wheel_t scroll_ctx;
    alps_scroll_wheel_init(&scroll_ctx);

    struct input_event raw_events[64];

    while (g_running) {
        uint64_t now = get_time_ms();
        int timeout_ms = -1;

        if (stick_fd >= 0) {
            /* Advance any deadlines that have already elapsed prior to sleeping */
            alps_action_t pre_act = alps_tap_gesture_handle_timeout(&tap_ctx, &cfg.tap, now, cfg.verbose);
            alps_emitter_dispatch(ufd, &pre_act);
            timeout_ms = alps_tap_gesture_get_timeout_ms(&tap_ctx, &cfg.tap, now);
        } else {
            /* Disconnected state: poll periodically as inotify safeguard */
            timeout_ms = 1000;
        }

        struct epoll_event ep_events[4];
        int nfds = epoll_wait(epfd, ep_events, 4, timeout_ms);

        if (nfds == 0) {
            if (stick_fd >= 0) {
                now = get_time_ms();
                alps_action_t tmo_act = alps_tap_gesture_handle_timeout(&tap_ctx, &cfg.tap, now, cfg.verbose);
                alps_emitter_dispatch(ufd, &tmo_act);
            } else {
                try_reconnect_device(&stick_fd, epfd, &cfg, dev_path, sizeof(dev_path), dev_name, sizeof(dev_name));
            }
            continue;
        }

        if (nfds < 0) {
            if (errno == EINTR) continue;
            perror("epoll_wait error");
            break;
        }

        for (int e = 0; e < nfds; e++) {
            int cur_fd = ep_events[e].data.fd;
            uint32_t cur_ev = ep_events[e].events;

            if (monitor_fd >= 0 && cur_fd == monitor_fd) {
                alps_device_monitor_drain(monitor_fd);
                if (stick_fd < 0) {
                    try_reconnect_device(&stick_fd, epfd, &cfg, dev_path, sizeof(dev_path), dev_name, sizeof(dev_name));
                }
            } else if (stick_fd >= 0 && cur_fd == stick_fd) {
                if (cur_ev & (EPOLLERR | EPOLLHUP)) {
                    handle_device_disconnect(&stick_fd, epfd, ufd, &tap_ctx, &scroll_ctx, cfg.verbose);
                    continue;
                }

                ssize_t bytes = read(stick_fd, raw_events, sizeof(raw_events));
                if (bytes < 0) {
                    if (errno == EAGAIN || errno == EINTR) continue;
                    handle_device_disconnect(&stick_fd, epfd, ufd, &tap_ctx, &scroll_ctx, cfg.verbose);
                    continue;
                }
                if (bytes == 0) {
                    handle_device_disconnect(&stick_fd, epfd, ufd, &tap_ctx, &scroll_ctx, cfg.verbose);
                    continue;
                }

                size_t count = (size_t)bytes / sizeof(struct input_event);
                now = get_time_ms();

                for (size_t i = 0; i < count; i++) {
                    alps_event_t norm_ev;
                    int recognized = 0;

                    if (raw_events[i].type == EV_REL) {
                        if (raw_events[i].code == REL_X) {
                            norm_ev.type = ALPS_EV_REL_X;
                            norm_ev.value = raw_events[i].value;
                            norm_ev.timestamp_ms = now;
                            recognized = 1;
                        } else if (raw_events[i].code == REL_Y) {
                            norm_ev.type = ALPS_EV_REL_Y;
                            norm_ev.value = raw_events[i].value;
                            norm_ev.timestamp_ms = now;
                            recognized = 1;
                        }
                    } else if (raw_events[i].type == EV_ABS && raw_events[i].code == ABS_PRESSURE) {
                        norm_ev.type = ALPS_EV_PRESSURE;
                        norm_ev.value = raw_events[i].value;
                        norm_ev.timestamp_ms = now;
                        recognized = 1;
                    } else if (raw_events[i].type == EV_KEY && raw_events[i].code == BTN_MIDDLE) {
                        norm_ev.type = ALPS_EV_BTN_MIDDLE;
                        norm_ev.value = raw_events[i].value;
                        norm_ev.timestamp_ms = now;
                        recognized = 1;
                    } else if (raw_events[i].type == EV_SYN && raw_events[i].code == SYN_REPORT) {
                        norm_ev.type = ALPS_EV_SYN_REPORT;
                        norm_ev.value = 0;
                        norm_ev.timestamp_ms = now;
                        recognized = 1;
                    }

                    if (recognized) {
                        if (norm_ev.type == ALPS_EV_BTN_MIDDLE) {
                            alps_action_t act = alps_scroll_wheel_process_event(&scroll_ctx, &cfg.scroll, &norm_ev, cfg.verbose);
                            alps_emitter_dispatch(ufd, &act);

                            if (alps_scroll_wheel_is_engaged(&scroll_ctx)) {
                                /* Reset tap gesture so middle-click doesn't leave an active tap state */
                                alps_action_t tap_reset = alps_tap_gesture_reset(&tap_ctx);
                                alps_emitter_dispatch(ufd, &tap_reset);
                            }
                        } else if (alps_scroll_wheel_is_engaged(&scroll_ctx)) {
                            /* While scrolling is engaged, pass motion and sync boundaries to the scroll module */
                            if (norm_ev.type == ALPS_EV_REL_X ||
                                norm_ev.type == ALPS_EV_REL_Y ||
                                norm_ev.type == ALPS_EV_SYN_REPORT) {
                                alps_action_t act = alps_scroll_wheel_process_event(&scroll_ctx, &cfg.scroll, &norm_ev, cfg.verbose);
                                alps_emitter_dispatch(ufd, &act);
                            }
                        } else {
                            /* Normal pointer navigation / tap gesture */
                            if (norm_ev.type != ALPS_EV_SYN_REPORT) {
                                alps_action_t act = alps_tap_gesture_process_event(&tap_ctx, &cfg.tap, &norm_ev, cfg.verbose);
                                alps_emitter_dispatch(ufd, &act);
                            }
                        }
                    }
                }
            }
        }
    }

    /* Guarantee virtual mouse button is released and resources freed on exit */
    handle_device_disconnect(&stick_fd, epfd, ufd, &tap_ctx, &scroll_ctx, cfg.verbose);
    if (monitor_fd >= 0) alps_device_monitor_destroy(monitor_fd);
    close(epfd);
    alps_emitter_destroy(ufd);
    return 0;
}
