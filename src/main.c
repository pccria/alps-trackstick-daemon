/**
 * @file main.c
 * @brief Daemon entry point, `epoll` event loop, and feature pipeline orchestrator.
 *
 * Responsibilities:
 * 1. Parse command-line configuration (`alps_config_parse_args`).
 * 2. Discover the ALPS DualPoint Stick (`/dev/input/event*`) and initialize
 *    the virtual `/dev/uinput` emitter (`alps_emitter_create`).
 * 3. Multiplex raw kernel input events via `epoll_wait` with dynamic timeouts
 *    queried from active feature modules.
 * 4. Normalize raw `struct input_event` packets into `alps_event_t` structs
 *    and route them through the feature pipeline (`alps_tap_gesture_process_event`).
 * 5. Guarantee clean virtual button release (`alps_tap_gesture_reset`) on
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

    char dev_path[512] = {0};
    char dev_name[256] = {0};
    int stick_fd = alps_device_find_stick(dev_path, sizeof(dev_path), dev_name, sizeof(dev_name));
    if (stick_fd < 0) {
        fprintf(stderr, "Error: ALPS DualPoint Stick not found!\n");
        return 1;
    }

    int ufd = alps_emitter_create();
    if (ufd < 0) {
        fprintf(stderr, "Error: Failed to initialize uinput device!\n");
        close(stick_fd);
        return 1;
    }

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        perror("epoll_create1 failed");
        alps_emitter_destroy(ufd);
        close(stick_fd);
        return 1;
    }

    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN;
    ev.data.fd = stick_fd;
    if (epoll_ctl(epfd, EPOLL_CTL_ADD, stick_fd, &ev) < 0) {
        perror("epoll_ctl failed");
        close(epfd);
        alps_emitter_destroy(ufd);
        close(stick_fd);
        return 1;
    }

    if (cfg.pointer_speed >= -1.0f && cfg.pointer_speed <= 1.0f) {
        alps_device_apply_pointer_speed(dev_name, cfg.pointer_speed);
    }

    if (cfg.verbose) {
        printf("ALPS Native C Daemon v%s (Modular Anti-Drift Transducer Engine)\n",
               ALPS_DAEMON_VERSION);
        printf("Connected: %s (%s)\n", dev_name, dev_path);
        if (cfg.pointer_speed >= -1.0f && cfg.pointer_speed <= 1.0f) {
            printf("Pointer Speed Override: %.2f (Forced Adaptive Profile)\n", cfg.pointer_speed);
        } else {
            printf("Pointer Ballistics: Native (KDE/System acceleration profile untouched)\n");
        }
        printf("Calibrated: TAP1_Z=%d, TAP2_Z=%d, RELEASE_Z=%d, TAP1_MAX_MOTION=%d, "
               "DRAG_MOTION=%d, DRAG_GRACE=%ums, ANTI_DRIFT=%s\n",
               cfg.tap.tap1_pressure, cfg.tap.tap2_rebound_pressure, cfg.tap.release_pressure,
               cfg.tap.max_tap_motion, cfg.tap.drag_start_motion, cfg.tap.drag_release_grace_ms,
               cfg.tap.anti_drift_enabled ? "ON" : "OFF");
        fflush(stdout);
    }

    alps_tap_gesture_t tap_ctx;
    alps_tap_gesture_init(&tap_ctx);

    struct input_event raw_events[64];

    while (g_running) {
        uint64_t now = get_time_ms();

        /* Advance any deadlines that have already elapsed prior to sleeping */
        alps_action_t pre_act = alps_tap_gesture_handle_timeout(&tap_ctx, &cfg.tap, now, cfg.verbose);
        alps_emitter_dispatch(ufd, &pre_act);

        int timeout_ms = alps_tap_gesture_get_timeout_ms(&tap_ctx, &cfg.tap, now);

        struct epoll_event ep_events[1];
        int nfds = epoll_wait(epfd, ep_events, 1, timeout_ms);

        if (nfds == 0) {
            now = get_time_ms();
            alps_action_t tmo_act = alps_tap_gesture_handle_timeout(&tap_ctx, &cfg.tap, now, cfg.verbose);
            alps_emitter_dispatch(ufd, &tmo_act);
            continue;
        }

        if (nfds < 0) {
            if (errno == EINTR) continue;
            perror("epoll_wait error");
            break;
        }

        if (ep_events[0].events & (EPOLLERR | EPOLLHUP)) {
            fprintf(stderr, "Input device disconnected (EPOLLERR/EPOLLHUP). Exiting for clean restart.\n");
            break;
        }

        ssize_t bytes = read(stick_fd, raw_events, sizeof(raw_events));
        if (bytes < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            perror("Device read error (disconnected)");
            break;
        }
        if (bytes == 0) {
            fprintf(stderr, "Input device EOF. Exiting.\n");
            break;
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
            }

            if (recognized) {
                alps_action_t act = alps_tap_gesture_process_event(&tap_ctx, &cfg.tap, &norm_ev, cfg.verbose);
                alps_emitter_dispatch(ufd, &act);
            }
        }
    }

    /* Guarantee virtual mouse button is released on exit */
    alps_action_t cleanup_act = alps_tap_gesture_reset(&tap_ctx);
    alps_emitter_dispatch(ufd, &cleanup_act);

    alps_emitter_destroy(ufd);
    close(stick_fd);
    close(epfd);
    return 0;
}
