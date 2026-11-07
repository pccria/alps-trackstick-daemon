/**
 * @file action.h
 * @brief Unified output action primitives emitted by feature modules.
 *
 * Feature modules (`tap_gesture`, and future modules like `scroll_wheel`) never
 * perform direct `/dev/uinput` system calls. Instead, they return an
 * `alps_action_t` describing the desired virtual mouse/scroll state transition,
 * which the I/O emitter layer translates into kernel `uinput` frames.
 */

#ifndef ALPS_CORE_ACTION_H
#define ALPS_CORE_ACTION_H

#include <stdint.h>

/**
 * @brief High-level virtual input action commands.
 */
typedef enum {
    ALPS_ACTION_NONE = 0,     /**< No output event required */
    ALPS_ACTION_CLICK,        /**< Discrete click: apply anti-drift, press BTN_LEFT, hold, release */
    ALPS_ACTION_BUTTON_DOWN,  /**< Immediate press: emit BTN_LEFT=1 (e.g., Tap 2 touchdown) */
    ALPS_ACTION_BUTTON_UP,    /**< Immediate release: apply optional anti-drift and emit BTN_LEFT=0 */
    ALPS_ACTION_SCROLL        /**< Emit vertical and/or horizontal wheel scroll events */
} alps_action_type_t;

/**
 * @brief Complete action descriptor passed from feature modules to the emitter.
 */
typedef struct {
    alps_action_type_t type; /**< Type of action to execute */
    int32_t drift_x;         /**< Anti-drift relative X correction (applied in same frame as click) */
    int32_t drift_y;         /**< Anti-drift relative Y correction (applied in same frame as click) */
    int32_t wheel_v;         /**< Standard vertical wheel notches (REL_WHEEL) */
    int32_t wheel_h;         /**< Standard horizontal wheel notches (REL_HWHEEL) */
    int32_t wheel_v_hires;   /**< High-resolution vertical scroll units (REL_WHEEL_HI_RES, 120 = 1 notch) */
    int32_t wheel_h_hires;   /**< High-resolution horizontal scroll units (REL_HWHEEL_HI_RES, 120 = 1 notch) */
    uint32_t hold_ms;        /**< Hold duration in milliseconds for ALPS_ACTION_CLICK */
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
        .wheel_h_hires = 0,
        .hold_ms = 0
    };
    return act;
}

#endif /* ALPS_CORE_ACTION_H */
