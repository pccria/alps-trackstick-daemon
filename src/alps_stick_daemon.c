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

#define DAEMON_VERSION               "0.4.0"

/* --- Calibrated Transducer Thresholds (v0.4.0 Anti-Drift Engine) --- */
#define TAP1_PRESSURE                22    /* Smooth, light activation floor: effortless tapping without hard whacking */
#define TAP2_REBOUND_PRESSURE        15    /* Sensitive rebound trigger: natural low-amplitude second strike */
#define RELEASE_PRESSURE             10    /* Mechanical unload baseline */
#define MAX_TAP_DURATION_MS          180   /* Maximum duration for an impulse tap */
#define MAX_TAP_MOTION               16    /* Maximum allowed lateral contact shear */
#define DOUBLE_TAP_WINDOW_MS         260   /* Window between Tap 1 release and Tap 2 touchdown */
#define AWAIT_MOTION_CANCEL          3     /* Navigation cancel: 3 counts aborts primed state on first movement packet */
#define DRAG_START_MOTION            6     /* Instant drag lock: 6 counts of steering locks drag without hesitation */
#define DRAG_START_HOLD_MS           160   /* Stationary hold duration on Tap 2 to engage drag */
#define DRAG_RELEASE_GRACE_MS        140   /* Crisp drop release: eliminates sticky drag linger */
#define CLICK_HOLD_MS                15    /* Minimum hold time in ms for discrete click registration */
#define PRE_MOTION_WINDOW_MS         150   /* Velocity memory window to detect active pointer navigation */
#define PRE_MOTION_LIMIT             10    /* Motion counts within window that classifies pressure as steering force */
#define MOTION_CAP                   10000 /* Saturation cap to prevent integer overflow */

typedef enum {
    STATE_IDLE,                  /* 0: Waiting for Tap 1 */
    STATE_TAP1_DOWN,             /* 1: Finger down on Tap 1 */
    STATE_AWAITING_TAP2,         /* 2: Tap 1 released, primed for Tap 2 */
    STATE_TAP2_DOWN,             /* 3: Finger down on Tap 2 (BTN_LEFT is immediately DOWN) */
    STATE_DRAGGING               /* 4: Active drag lock (BTN_LEFT held while steering) */
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
        ioctl(ufd, UI_SET_KEYBIT, BTN_LEFT) < 0 ||
        ioctl(ufd, UI_SET_EVBIT, EV_REL) < 0 ||
        ioctl(ufd, UI_SET_RELBIT, REL_X) < 0 ||
        ioctl(ufd, UI_SET_RELBIT, REL_Y) < 0) {
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
    float speed = -999.0f; /* Unset by default: leave desktop environment / KDE settings completely untouched */

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

    if (speed >= -1.0f && speed <= 1.0f) {
        apply_pointer_speed(dev_name, speed);
    }

    if (verbose) {
        printf("ALPS Native C Daemon v%s (Anti-Drift Transducer Engine)\n", DAEMON_VERSION);
        printf("Connected: %s (%s)\n", dev_name, dev_path);
        if (speed >= -1.0f && speed <= 1.0f) {
            printf("Pointer Speed Override: %.2f (Forced Adaptive Profile)\n", speed);
        } else {
            printf("Pointer Ballistics: Native (KDE/System acceleration profile untouched)\n");
        }
        printf("Calibrated: TAP1_Z=%d, TAP2_Z=%d, RELEASE_Z=%d, TAP1_MAX_MOTION=%d, DRAG_MOTION=%d, DRAG_GRACE=%dms\n",
               TAP1_PRESSURE, TAP2_REBOUND_PRESSURE, RELEASE_PRESSURE,
               MAX_TAP_MOTION, DRAG_START_MOTION, DRAG_RELEASE_GRACE_MS);
        fflush(stdout);
    }

    state_t state = STATE_IDLE;
    uint64_t press_time = 0;
    uint64_t tap1_release_time = 0;
    uint64_t last_active_time = 0;
    uint64_t last_rel_time = 0;
    int max_pressure = 0;
    int tap1_motion = 0;
    int tap1_dx = 0;
    int tap1_dy = 0;
    int tap2_motion = 0;
    int tap2_dx = 0;
    int tap2_dy = 0;
    int await_motion = 0;
    int pre_motion = 0;

    struct input_event evs[64];

    while (g_running) {
        uint64_t now = get_time_ms();
        int timeout_ms = -1;

        if (state == STATE_TAP1_DOWN) {
            uint64_t held = now >= press_time ? (now - press_time) : 0;
            if (held >= (MAX_TAP_DURATION_MS + 50)) {
                state = STATE_IDLE;
            } else {
                timeout_ms = (int)((MAX_TAP_DURATION_MS + 50) - held);
            }
        } else if (state == STATE_AWAITING_TAP2) {
            uint64_t elapsed = now >= tap1_release_time ? (now - tap1_release_time) : 0;
            if (elapsed >= DOUBLE_TAP_WINDOW_MS) {
                state = STATE_IDLE;
            } else {
                timeout_ms = (int)(DOUBLE_TAP_WINDOW_MS - elapsed);
            }
        } else if (state == STATE_TAP2_DOWN) {
            uint64_t held = now >= press_time ? (now - press_time) : 0;
            if (held >= DRAG_START_HOLD_MS) {
                state = STATE_DRAGGING;
                last_active_time = now;
                if (verbose) printf("  [DRAG ENGAGED: Stationary Hold (%lums)]\n", (unsigned long)held);
            } else {
                timeout_ms = (int)(DRAG_START_HOLD_MS - held);
            }
        } else if (state == STATE_DRAGGING) {
            uint64_t inactive = now >= last_active_time ? (now - last_active_time) : 0;
            if (inactive >= DRAG_RELEASE_GRACE_MS) {
                emit_up(ufd);
                state = STATE_IDLE;
                if (verbose) printf("  [DRAG RELEASED: Inactivity Grace (%dms)]\n", DRAG_RELEASE_GRACE_MS);
            } else {
                timeout_ms = (int)(DRAG_RELEASE_GRACE_MS - inactive);
            }
        }

        struct epoll_event events[1];
        int nfds = epoll_wait(epfd, events, 1, timeout_ms);

        if (nfds == 0) {
            now = get_time_ms();
            if (state == STATE_TAP1_DOWN) {
                if (now >= press_time && (now - press_time) >= (MAX_TAP_DURATION_MS + 50)) {
                    state = STATE_IDLE;
                }
            } else if (state == STATE_AWAITING_TAP2) {
                if (now >= tap1_release_time && (now - tap1_release_time) >= DOUBLE_TAP_WINDOW_MS) {
                    state = STATE_IDLE;
                }
            } else if (state == STATE_TAP2_DOWN) {
                if (now >= press_time && (now - press_time) >= DRAG_START_HOLD_MS) {
                    state = STATE_DRAGGING;
                    last_active_time = now;
                    if (verbose) printf("  [DRAG ENGAGED: Stationary Hold Timeout]\n");
                }
            } else if (state == STATE_DRAGGING) {
                if (now >= last_active_time && (now - last_active_time) >= DRAG_RELEASE_GRACE_MS) {
                    emit_up(ufd);
                    state = STATE_IDLE;
                    if (verbose) printf("  [DRAG RELEASED: Inactivity Timeout]\n");
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
            /* 1. Lateral motion events (EV_REL) */
            if (evs[i].type == EV_REL) {
                /* Smooth decaying pre-press velocity tracker */
                uint64_t rel_dt = now >= last_rel_time ? (now - last_rel_time) : 0;
                if (rel_dt > PRE_MOTION_WINDOW_MS) {
                    pre_motion = 0;
                } else if (rel_dt > 60) {
                    pre_motion /= 2;
                }
                pre_motion = add_motion_saturated(pre_motion, evs[i].value);
                last_rel_time = now;

                if (state == STATE_TAP1_DOWN) {
                    tap1_motion = add_motion_saturated(tap1_motion, evs[i].value);
                    if (evs[i].code == REL_X) tap1_dx += evs[i].value;
                    else if (evs[i].code == REL_Y) tap1_dy += evs[i].value;
                } else if (state == STATE_AWAITING_TAP2) {
                    await_motion = add_motion_saturated(await_motion, evs[i].value);
                    if (await_motion >= AWAIT_MOTION_CANCEL) {
                        /* Active pointer steering: cancel primed state immediately on first movement */
                        state = STATE_IDLE;
                        if (verbose) printf("  [AWAIT CANCELLED: Navigation Steering (%d counts)]\n", await_motion);
                    }
                } else if (state == STATE_TAP2_DOWN) {
                    tap2_motion = add_motion_saturated(tap2_motion, evs[i].value);
                    if (evs[i].code == REL_X) tap2_dx += evs[i].value;
                    else if (evs[i].code == REL_Y) tap2_dy += evs[i].value;

                    if (tap2_motion >= DRAG_START_MOTION) {
                        state = STATE_DRAGGING;
                        last_active_time = now;
                        if (verbose) printf("  [DRAG ENGAGED: Instant Steering Lock (%d counts)]\n", tap2_motion);
                    }
                } else if (state == STATE_DRAGGING) {
                    last_active_time = now;
                }
            }
            /* 2. Downward pressure events (EV_ABS, ABS_PRESSURE) */
            else if (evs[i].type == EV_ABS && evs[i].code == ABS_PRESSURE) {
                int val = evs[i].value;

                /* --- TOUCH DOWN --- */
                if (state == STATE_IDLE) {
                    if (val >= TAP1_PRESSURE) {
                        /* Ignore pressure spikes if stick was actively moving (steering force) */
                        uint64_t rel_elapsed = now >= last_rel_time ? (now - last_rel_time) : 0;
                        if (rel_elapsed <= PRE_MOTION_WINDOW_MS && pre_motion >= PRE_MOTION_LIMIT) {
                            if (verbose) printf("  [TAP 1 IGNORED: Steering Velocity (pre_motion=%d, Z=%d)]\n", pre_motion, val);
                        } else {
                            state = STATE_TAP1_DOWN;
                            press_time = now;
                            max_pressure = val;
                            tap1_motion = 0;
                            tap1_dx = 0;
                            tap1_dy = 0;
                        }
                    }
                } else if (state == STATE_TAP1_DOWN) {
                    if (val > max_pressure) max_pressure = val;
                } else if (state == STATE_AWAITING_TAP2) {
                    if (val >= TAP2_REBOUND_PRESSURE) {
                        /* Instant virtual button down on second contact */
                        state = STATE_TAP2_DOWN;
                        press_time = now;
                        tap2_motion = 0;
                        tap2_dx = 0;
                        tap2_dy = 0;
                        emit_down(ufd);
                        if (verbose) printf("  [TAP 2 CONTACT: Pressure=%d -> BTN_LEFT DOWN]\n", val);
                    }
                } else if (state == STATE_DRAGGING) {
                    if (val > RELEASE_PRESSURE) {
                        last_active_time = now;
                    }
                }

                /* --- LIFT OFF / PRESSURE UNLOAD --- */
                if (val <= RELEASE_PRESSURE) {
                    if (state == STATE_TAP1_DOWN) {
                        uint64_t duration = now >= press_time ? (now - press_time) : 0;
                        /* Physical Discriminator: Must be brief, under motion cap, AND dominated by vertical force */
                        int is_valid_tap = (duration <= MAX_TAP_DURATION_MS) &&
                                           (tap1_motion <= MAX_TAP_MOTION) &&
                                           (tap1_motion <= 6 || max_pressure >= (tap1_motion * 3 / 2));

                        if (is_valid_tap) {
                            /* Anti-drift snap: reverse contact shear before clicking so cursor stays rock-solid on target */
                            if (tap1_dx != 0) emit_event(ufd, EV_REL, REL_X, -tap1_dx);
                            if (tap1_dy != 0) emit_event(ufd, EV_REL, REL_Y, -tap1_dy);

                            emit_down(ufd);
                            usleep(CLICK_HOLD_MS * 1000);
                            emit_up(ufd);
                            tap1_release_time = now;
                            state = STATE_AWAITING_TAP2;
                            await_motion = 0;
                            if (verbose) printf("-> TAP 1 CLICK (duration=%lums, motion=%d, drift=[%+d, %+d], max_Z=%d)\n",
                                                (unsigned long)duration, tap1_motion, tap1_dx, tap1_dy, max_pressure);
                        } else {
                            state = STATE_IDLE;
                            if (verbose) printf("  [TAP 1 REJECTED: duration=%lums, motion=%d, max_Z=%d (ratio check)]\n",
                                                (unsigned long)duration, tap1_motion, max_pressure);
                        }
                    } else if (state == STATE_TAP2_DOWN) {
                        uint64_t duration = now >= press_time ? (now - press_time) : 0;
                        if (duration <= MAX_TAP_DURATION_MS && tap2_motion < DRAG_START_MOTION) {
                            /* Discrete second tap: ensure minimum hold time, reverse drift, and release immediately */
                            if (duration < CLICK_HOLD_MS) {
                                usleep((useconds_t)(CLICK_HOLD_MS - duration) * 1000);
                            }
                            if (tap2_dx != 0) emit_event(ufd, EV_REL, REL_X, -tap2_dx);
                            if (tap2_dy != 0) emit_event(ufd, EV_REL, REL_Y, -tap2_dy);

                            emit_up(ufd);
                            state = STATE_IDLE;
                            if (verbose) printf("-> DOUBLE CLICK COMPLETED (Tap 2 duration=%lums, motion=%d, drift=[%+d, %+d])\n",
                                                (unsigned long)duration, tap2_motion, tap2_dx, tap2_dy);
                        } else {
                            /* Held or moved: lock into drag */
                            state = STATE_DRAGGING;
                            last_active_time = now;
                            if (verbose) printf("  [DRAG ENGAGED: Sustained Tap 2 (motion=%d)]\n", tap2_motion);
                        }
                    }
                    /* In STATE_DRAGGING: pressure drop to 0 is expected when steering laterally;
                       drag stays engaged via relative motion stream and inactivity grace */
                }
            }
        }
    }

    if (state == STATE_DRAGGING || state == STATE_TAP2_DOWN) {
        emit_up(ufd);
    }

    ioctl(ufd, UI_DEV_DESTROY);
    close(ufd);
    close(stick_fd);
    close(epfd);
    return 0;
}
