/**
 * @file emitter.c
 * @brief Implementation of the `/dev/uinput` virtual mouse and scroll emitter.
 *
 * All action dispatches are strictly non-blocking. Button down/up transitions
 * and anti-drift relative motions are written directly to the uinput file descriptor
 * with zero thread-blocking sleeps.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "io/emitter.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <linux/uinput.h>

/**
 * @brief Writes a single raw `struct input_event` to the `uinput` descriptor.
 */
static void emit_raw(int ufd, uint16_t type, uint16_t code, int32_t val) {
    struct input_event ie;
    memset(&ie, 0, sizeof(ie));
    ie.type = type;
    ie.code = code;
    ie.value = val;
    ssize_t res = write(ufd, &ie, sizeof(ie));
    (void)res;
}

/**
 * @brief Flushes the current `uinput` packet frame via `SYN_REPORT`.
 */
static void emit_syn(int ufd) {
    emit_raw(ufd, EV_SYN, SYN_REPORT, 0);
}

int alps_emitter_create(void) {
    int ufd = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
    if (ufd < 0) {
        ufd = open("/dev/input/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
        if (ufd < 0) return -1;
    }

    if (ioctl(ufd, UI_SET_EVBIT, EV_KEY) < 0 ||
        ioctl(ufd, UI_SET_KEYBIT, BTN_LEFT) < 0 ||
        ioctl(ufd, UI_SET_EVBIT, EV_REL) < 0 ||
        ioctl(ufd, UI_SET_RELBIT, REL_X) < 0 ||
        ioctl(ufd, UI_SET_RELBIT, REL_Y) < 0 ||
        ioctl(ufd, UI_SET_RELBIT, REL_WHEEL) < 0 ||
        ioctl(ufd, UI_SET_RELBIT, REL_HWHEEL) < 0) {
        close(ufd);
        return -1;
    }

#ifdef REL_WHEEL_HI_RES
    (void)ioctl(ufd, UI_SET_RELBIT, REL_WHEEL_HI_RES);
#endif
#ifdef REL_HWHEEL_HI_RES
    (void)ioctl(ufd, UI_SET_RELBIT, REL_HWHEEL_HI_RES);
#endif

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

void alps_emitter_destroy(int ufd) {
    if (ufd < 0) return;
    (void)ioctl(ufd, UI_DEV_DESTROY);
    close(ufd);
}

void alps_emitter_dispatch(int ufd, const alps_action_t *action) {
    if (ufd < 0 || !action || action->type == ALPS_ACTION_NONE) {
        return;
    }

    switch (action->type) {
        case ALPS_ACTION_BUTTON_DOWN:
            /* Emit anti-drift relative motion snap in the same frame as button down */
            if (action->drift_x != 0) {
                emit_raw(ufd, EV_REL, REL_X, action->drift_x);
            }
            if (action->drift_y != 0) {
                emit_raw(ufd, EV_REL, REL_Y, action->drift_y);
            }
            emit_raw(ufd, EV_KEY, BTN_LEFT, 1);
            emit_syn(ufd);
            break;

        case ALPS_ACTION_BUTTON_UP:
            if (action->drift_x != 0) {
                emit_raw(ufd, EV_REL, REL_X, action->drift_x);
            }
            if (action->drift_y != 0) {
                emit_raw(ufd, EV_REL, REL_Y, action->drift_y);
            }
            emit_raw(ufd, EV_KEY, BTN_LEFT, 0);
            emit_syn(ufd);
            break;

        case ALPS_ACTION_CLICK:
            /* Atomic single-frame click */
            if (action->drift_x != 0) {
                emit_raw(ufd, EV_REL, REL_X, action->drift_x);
            }
            if (action->drift_y != 0) {
                emit_raw(ufd, EV_REL, REL_Y, action->drift_y);
            }
            emit_raw(ufd, EV_KEY, BTN_LEFT, 1);
            emit_raw(ufd, EV_KEY, BTN_LEFT, 0);
            emit_syn(ufd);
            break;

        case ALPS_ACTION_SCROLL: {
            int emitted = 0;
            if (action->wheel_v != 0) {
                emit_raw(ufd, EV_REL, REL_WHEEL, action->wheel_v);
                emitted = 1;
            }
            if (action->wheel_h != 0) {
                emit_raw(ufd, EV_REL, REL_HWHEEL, action->wheel_h);
                emitted = 1;
            }
#ifdef REL_WHEEL_HI_RES
            if (action->wheel_v_hires != 0) {
                emit_raw(ufd, EV_REL, REL_WHEEL_HI_RES, action->wheel_v_hires);
                emitted = 1;
            }
#endif
#ifdef REL_HWHEEL_HI_RES
            if (action->wheel_h_hires != 0) {
                emit_raw(ufd, EV_REL, REL_HWHEEL_HI_RES, action->wheel_h_hires);
                emitted = 1;
            }
#endif
            if (emitted) {
                emit_syn(ufd);
            }
            break;
        }

        case ALPS_ACTION_NONE:
        default:
            break;
    }
}
