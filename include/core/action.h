/**
 * @file action.h
 * @brief Unified non-blocking output action primitives emitted by feature modules.
 *
 * Feature modules (`tap_gesture`, and future modules like `scroll_wheel`) never
 * perform direct `/dev/uinput` system calls or blocking sleeps. Instead, they
 * return an `alps_action_t` describing instantaneous virtual mouse/scroll state
 * transitions, which the I/O emitter layer translates into atomic kernel `uinput` frames.
 */

#ifndef ALPS_CORE_ACTION_H
#define ALPS_CORE_ACTION_H

#include <stdint.h>

/**
 * @brief High-level virtual input action commands.
 */
typedef enum {
    ALPS_ACTION_NONE = 0,     /**< No output event required */
    ALPS_ACTION_CLICK,        /**< Atomic single-frame click: optional anti-drift, press, release */
    ALPS_ACTION_BUTTON_DOWN,  /**< Immediate press: optional anti-drift and BTN_LEFT=1 */
    ALPS_ACTION_BUTTON_UP,    /**< Immediate release: optional anti-drift and BTN_LEFT=0 */
    ALPS_ACTION_SCROLL        /**< Emit vertical and/or horizontal wheel scroll events */
} alps_action_type_t;

/**
 * @brief Complete action descriptor passed from feature modules to the emitter.
 *
 * All actions are 100% non-blocking; any click hold durations or timing intervals
 * are evaluated strictly via epoll_wait deadlines in the feature state machine.
 */
typedef struct {
    alps_action_type_t type; /**< Type of action to execute */
    int32_t drift_x;         /**< Anti-drift relative X correction (applied in same frame) */
    int32_t drift_y;         /**< Anti-drift relative Y correction (applied in same frame) */
    int32_t wheel_v;         /**< Standard vertical wheel notches (REL_WHEEL) */
    int32_t wheel_h;         /**< Standard horizontal wheel notches (REL_HWHEEL) */
    int32_t wheel_v_hires;   /**< High-resolution vertical scroll units (REL_WHEEL_HI_RES, 120 = 1 notch) */
    int32_t wheel_h_hires;   /**< High-resolution horizontal scroll units (REL_HWHEEL_HI_RES, 120 = 1 notch) */
} alps_action_t;

/**
 * @brief Returns a zero-initialized `ALPS_ACTION_NONE` descriptor.
 */
static inline alps_action_t alps_action_none(void) {
    alps_action_t act = {
        .type = ALPS_ACTION_NONE,
        .drift_x = 0,
        .drift_y = 0,
        .wheel_v = 0,
        .wheel_h = 0,
        .wheel_v_hires = 0,
        .wheel_h_hires = 0
    };
    return act;
}

#endif /* ALPS_CORE_ACTION_H */
