#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <time.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <linux/input.h>
#include <linux/uinput.h>

#define DAEMON_VERSION          "1.2.0"

/* --- Calibrated Thresholds --- */
#define TAP_PRESSURE            22    /* Gentle touch threshold */
#define RELEASE_PRESSURE        10    /* Baseline release pressure */
#define MAX_TAP_DURATION_MS     220   /* Max bounce time for an impulse tap */
#define MAX_TAP_MOTION          35    /* Tremor limit for taps (accommodates ~20 counts of physical contact shear) */
#define DOUBLE_TAP_WINDOW_MS    450   /* Generous double-tap window matching desktop standards */
#define DRAG_START_HOLD_MS      180   /* Hold time on Tap 2 before engaging drag lock if stationary */
#define DRAG_RELEASE_GRACE_MS   180   /* Silence window (no motion + no pressure) to release drag */
#define MOTION_CAP              10000 /* Saturation cap to prevent integer overflow */

typedef enum {
    STATE_IDLE,
    STATE_TAP1_DOWN,
    STATE_AWAITING_TAP2,
    STATE_TAP2_DOWN,
    STATE_DRAGGING
} state_t;

static volatile sig_atomic_t g_running = 1;

static void sig_handler(int sig) {
    (void)sig;
    g_running = 0;
}

static int setup_signals(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; /* Do not set SA_RESTART so epoll_wait breaks cleanly */

    if (sigaction(SIGINT, &sa, NULL) < 0) return -1;
    if (sigaction(SIGTERM, &sa, NULL) < 0) return -1;
    return 0;
}

static uint64_t get_time_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0) {
        return 0;
    }
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
}

/* Safe saturating motion accumulator: avoids abs(INT32_MIN) UB and integer overflow */
static int add_motion_saturated(int current, int32_t delta) {
    int32_t mag;
    if (delta == INT32_MIN) {
        mag = MOTION_CAP;
    } else {
        mag = delta < 0 ? -delta : delta;
    }
    if (mag > MOTION_CAP) {
        mag = MOTION_CAP;
    }
    if (current > MOTION_CAP - mag) {
        return MOTION_CAP;
    }
    return current + (int)mag;
}

static int find_stick_device(char *out_path, size_t max_len, char *out_name, size_t name_max) {
    DIR *dir = opendir("/dev/input");
    if (!dir) return -1;

    struct dirent *ent;
    int found_fd = -1;

    while ((ent = readdir(dir)) != NULL) {
        if (strncmp(ent->d_name, "event", 5) != 0) continue;

        char path[512];
        int n = snprintf(path, sizeof(path), "/dev/input/%s", ent->d_name);
        if (n < 0 || (size_t)n >= sizeof(path)) continue;

        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;

        char name[256];
        memset(name, 0, sizeof(name));
        if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0) {
            name[sizeof(name) - 1] = '\0';
            if (strstr(name, "DualPoint Stick") != NULL) {
                snprintf(out_path, max_len, "%s", path);
                snprintf(out_name, name_max, "%s", name);
                found_fd = fd;
                break;
            }
        }
        close(fd);
    }
    closedir(dir);
    return found_fd;
}

static int create_uinput_device(void) {
    int ufd = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
    if (ufd < 0) {
        ufd = open("/dev/input/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
        if (ufd < 0) return -1;
    }

    if (ioctl(ufd, UI_SET_EVBIT, EV_KEY) < 0 ||
        ioctl(ufd, UI_SET_KEYBIT, BTN_LEFT) < 0) {
        close(ufd);
        return -1;
    }

    struct uinput_setup usetup;
    memset(&usetup, 0, sizeof(usetup));
    usetup.id.bustype = BUS_USB;
    usetup.id.vendor = 0x1234;
    usetup.id.product = 0x5678;
    snprintf(usetup.name, sizeof(usetup.name), "%s", "alps-tap-emitter");

    if (ioctl(ufd, UI_DEV_SETUP, &usetup) < 0 || ioctl(ufd, UI_DEV_CREATE) < 0) {
        close(ufd);
        return -1;
    }
    return ufd;
}

static void emit_event(int ufd, uint16_t type, uint16_t code, int32_t val) {
    struct input_event ie;
    memset(&ie, 0, sizeof(ie));
    ie.type = type;
    ie.code = code;
    ie.value = val;
    ssize_t res = write(ufd, &ie, sizeof(ie));
    (void)res;
}

static void emit_syn(int ufd) {
    emit_event(ufd, EV_SYN, SYN_REPORT, 0);
}

static void emit_down(int ufd) {
    emit_event(ufd, EV_KEY, BTN_LEFT, 1);
    emit_syn(ufd);
}

static void emit_up(int ufd) {
    emit_event(ufd, EV_KEY, BTN_LEFT, 0);
    emit_syn(ufd);
}

static void emit_quick_click(int ufd) {
    emit_down(ufd);
    usleep(5000); /* 5ms hold: distinct events in kernel queue without long epoll stall */
    emit_up(ufd);
}

/* Safe execution without shell invocation (prevents command injection & PATH hijacking) */
static void run_xinput_cmd(char *const argv[]) {
    pid_t pid = fork();
    if (pid < 0) return;

    if (pid == 0) {
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        execv("/usr/bin/xinput", argv);
        _exit(127);
    } else {
        int status = 0;
        waitpid(pid, &status, 0);
    }
}

static void apply_pointer_speed(const char *dev_name, float speed) {
    if (speed < -1.0f || speed > 1.0f) return;

    char speed_str[32];
    snprintf(speed_str, sizeof(speed_str), "%.2f", speed);

    char *const args_speed[] = {
        "xinput", "set-prop", (char *)dev_name, "libinput Accel Speed", speed_str, NULL
    };
    run_xinput_cmd(args_speed);

    char *const args_profile[] = {
        "xinput", "set-prop", (char *)dev_name, "libinput Accel Profile Enabled", "1", "0", NULL
    };
    run_xinput_cmd(args_profile);
}

int main(int argc, char *argv[]) {
    int verbose = 0;
    float speed = 0.8f;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            verbose = 1;
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--speed") == 0) {
            if (i + 1 < argc) {
                char *endptr = NULL;
                errno = 0;
                float val = strtof(argv[++i], &endptr);
                if (errno == 0 && endptr != argv[i] && *endptr == '\0' && val >= -1.0f && val <= 1.0f) {
                    speed = val;
                } else {
                    fprintf(stderr, "Invalid speed value (must be between -1.0 and 1.0)\n");
                    return 1;
                }
            }
        }
    }

    if (setup_signals() < 0) {
        perror("Failed to configure signals");
        return 1;
    }

    char dev_path[512] = {0};
    char dev_name[256] = {0};
    int stick_fd = find_stick_device(dev_path, sizeof(dev_path), dev_name, sizeof(dev_name));
    if (stick_fd < 0) {
        fprintf(stderr, "Error: ALPS DualPoint Stick not found!\n");
        return 1;
    }

    int ufd = create_uinput_device();
    if (ufd < 0) {
        fprintf(stderr, "Error: Failed to initialize uinput device!\n");
        close(stick_fd);
        return 1;
    }

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        perror("epoll_create1 failed");
        ioctl(ufd, UI_DEV_DESTROY);
        close(ufd);
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
        ioctl(ufd, UI_DEV_DESTROY);
        close(ufd);
        close(stick_fd);
        return 1;
    }

    apply_pointer_speed(dev_name, speed);

    if (verbose) {
        printf("ALPS Native C Daemon (Motion-Coupled Drag Lock)\n");
        printf("Connected: %s (%s)\n", dev_name, dev_path);
        fflush(stdout);
    }

    state_t state = STATE_IDLE;
    uint64_t press_time = 0;
    uint64_t tap1_release_time = 0;
    uint64_t last_active_time = 0;
    int max_pressure = 0;
    int tap1_motion = 0;
    int tap2_motion = 0;

    struct input_event evs[64];

    while (g_running) {
        uint64_t now = get_time_ms();
        int timeout_ms = -1;

        if (state == STATE_AWAITING_TAP2) {
            uint64_t elapsed = now >= tap1_release_time ? (now - tap1_release_time) : 0;
            if (elapsed >= DOUBLE_TAP_WINDOW_MS) {
                state = STATE_IDLE;
            } else {
                timeout_ms = (int)(DOUBLE_TAP_WINDOW_MS - elapsed);
            }
        } else if (state == STATE_TAP2_DOWN) {
            /* Hold timer: if user keeps finger stationary on Tap 2, engage drag */
            uint64_t hold = now >= press_time ? (now - press_time) : 0;
            if (hold >= DRAG_START_HOLD_MS) {
                emit_down(ufd);
                last_active_time = now;
                state = STATE_DRAGGING;
                if (verbose) printf("  [DRAG ENGAGED: Hold Threshold Reached (%lums)]\n", (unsigned long)hold);
            } else {
                timeout_ms = (int)(DRAG_START_HOLD_MS - hold);
            }
        } else if (state == STATE_DRAGGING) {
            /* Drag release timer: releases after sustained silence (no motion and no pressure) */
            uint64_t inactive = now >= last_active_time ? (now - last_active_time) : 0;
            if (inactive >= DRAG_RELEASE_GRACE_MS) {
                emit_up(ufd);
                state = STATE_IDLE;
                if (verbose) printf("  [DRAG COMPLETED: Released after silence (%d ms)]\n", DRAG_RELEASE_GRACE_MS);
            } else {
                timeout_ms = (int)(DRAG_RELEASE_GRACE_MS - inactive);
            }
        }

        struct epoll_event events[1];
        int nfds = epoll_wait(epfd, events, 1, timeout_ms);

        if (nfds == 0) {
            now = get_time_ms();
            if (state == STATE_AWAITING_TAP2) {
                if (now >= tap1_release_time && (now - tap1_release_time) >= DOUBLE_TAP_WINDOW_MS) {
                    state = STATE_IDLE;
                }
            } else if (state == STATE_TAP2_DOWN) {
                if (now >= press_time && (now - press_time) >= DRAG_START_HOLD_MS) {
                    emit_down(ufd);
                    last_active_time = now;
                    state = STATE_DRAGGING;
                    if (verbose) printf("  [DRAG ENGAGED: Hold Threshold Reached]\n");
                }
            } else if (state == STATE_DRAGGING) {
                if (now >= last_active_time && (now - last_active_time) >= DRAG_RELEASE_GRACE_MS) {
                    emit_up(ufd);
                    state = STATE_IDLE;
                    if (verbose) printf("  [DRAG COMPLETED: Released after silence]\n");
                }
            }
            continue;
        }

        if (nfds < 0) {
            if (errno == EINTR) continue;
            perror("epoll_wait error");
            break;
        }

        /* Detect device disconnect / hangup (e.g. suspend/resume or module reload) */
        if (events[0].events & (EPOLLERR | EPOLLHUP)) {
            fprintf(stderr, "Input device disconnected (EPOLLERR/EPOLLHUP). Exiting for clean restart.\n");
            break;
        }

        ssize_t bytes = read(stick_fd, evs, sizeof(evs));
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
            /* 1. Lateral motion events */
            if (evs[i].type == EV_REL) {
                if (state == STATE_TAP1_DOWN) {
                    tap1_motion = add_motion_saturated(tap1_motion, evs[i].value);
                } else if (state == STATE_TAP2_DOWN) {
                    tap2_motion = add_motion_saturated(tap2_motion, evs[i].value);
                    /* 
                     * If steering motion exceeds the tap tremor threshold, user is actively
                     * dragging across the desktop. Engage drag immediately!
                     */
                    if (tap2_motion > MAX_TAP_MOTION) {
                        emit_down(ufd);
                        last_active_time = now;
                        state = STATE_DRAGGING;
                        if (verbose) printf("  [DRAG ENGAGED: Steering Motion Detected (%d counts)]\n", tap2_motion);
                    }
                } else if (state == STATE_DRAGGING) {
                    /* Active movement keeps the drag firmly locked */
                    last_active_time = now;
                }
            }
            /* 2. Downward pressure events */
            else if (evs[i].type == EV_ABS && evs[i].code == ABS_PRESSURE) {
                int val = evs[i].value;

                /* --- TOUCH DOWN --- */
                if (val >= TAP_PRESSURE) {
                    if (state == STATE_IDLE) {
                        state = STATE_TAP1_DOWN;
                        press_time = now;
                        max_pressure = val;
                        tap1_motion = 0;
                    } else if (state == STATE_TAP1_DOWN) {
                        if (val > max_pressure) max_pressure = val;
                    } else if (state == STATE_AWAITING_TAP2) {
                        /* Second tap contact: arm Tap 2 */
                        state = STATE_TAP2_DOWN;
                        press_time = now;
                        tap2_motion = 0;
                        if (verbose) printf("  [TAP 2 CONTACT: Awaiting Lift-Off or Drag]\n");
                    } else if (state == STATE_DRAGGING) {
                        last_active_time = now;
                    }
                }
                /* --- HOLD: Resting pressure maintains active drag --- */
                else if (val > RELEASE_PRESSURE) {
                    if (state == STATE_DRAGGING) {
                        last_active_time = now;
                    }
                }
                /* --- LIFT OFF / PRESSURE DIP --- */
                else if (val <= RELEASE_PRESSURE) {
                    if (state == STATE_TAP1_DOWN) {
                        uint64_t duration = now >= press_time ? (now - press_time) : 0;
                        if (duration <= MAX_TAP_DURATION_MS && tap1_motion <= MAX_TAP_MOTION) {
                            emit_quick_click(ufd);
                            tap1_release_time = now;
                            state = STATE_AWAITING_TAP2;
                            if (verbose) printf("-> TAP 1 (Click!)\n");
                        } else {
                            state = STATE_IDLE;
                        }
                    } else if (state == STATE_TAP2_DOWN) {
                        uint64_t duration = now >= press_time ? (now - press_time) : 0;
                        /* 
                         * Lifted off quickly without sustained steering: pure Double Click!
                         * Emits second clean click instantly without drag sticking.
                         */
                        if (duration <= MAX_TAP_DURATION_MS && tap2_motion <= MAX_TAP_MOTION) {
                            emit_quick_click(ufd);
                            state = STATE_IDLE;
                            if (verbose) printf("-> DOUBLE CLICK Completed!\n");
                        } else {
                            state = STATE_IDLE;
                        }
                    }
                    /* If STATE_DRAGGING: pressure dip is expected during steering; silence timer handles release */
                }
            }
        }
    }

    if (state == STATE_DRAGGING) emit_up(ufd);

    ioctl(ufd, UI_DEV_DESTROY);
    close(ufd);
    close(stick_fd);
    close(epfd);
    return 0;
}
