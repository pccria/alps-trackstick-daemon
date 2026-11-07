/**
 * @file scroll_wheel.h
 * @brief High-resolution 2D kinetic trackstick scroll wheel state machine.
 *
 * Implements a pure, zero-I/O finite state machine that intercepts physical
 * middle-button hold events (`BTN_MIDDLE`) and translates relative stick deflections
 * (`REL_X`, `REL_Y`) into continuous high-resolution vertical (`REL_WHEEL_HI_RES`)
 * and horizontal (`REL_HWHEEL_HI_RES`) scroll actions (`alps_action_t`), with
 * sub-pixel remainder accumulation and primary axis-lock deadband filtering.
 */

#ifndef ALPS_FEATURES_SCROLL_WHEEL_H
#define ALPS_FEATURES_SCROLL_WHEEL_H

#include <stdint.h>
#include "core/config.h"
#include "core/event.h"
#include "core/action.h"

/**
 * @brief Internal lifecycle states of the middle-button scroll machine.
 */
typedef enum {
    ALPS_SCROLL_STATE_IDLE = 0,    /**< Middle button not pressed; relative motion passes to cursor */
    ALPS_SCROLL_STATE_ENGAGED      /**< Middle button held down; relative motion converted to scrolling */
} alps_scroll_state_t;

/**
 * @brief Directional axis-lock mode to prevent unintended lateral wheel jitter.
 */
typedef enum {
    ALPS_AXIS_LOCK_NONE = 0,       /**< No axis locked; evaluating dominant initial deflection */
    ALPS_AXIS_LOCK_VERTICAL,       /**< Locked to pure vertical scroll (horizontal deadband enforced) */
    ALPS_AXIS_LOCK_HORIZONTAL      /**< Locked to pure horizontal scroll (vertical deadband enforced) */
} alps_axis_lock_t;

/**
 * @brief Complete state context for the 2D scroll wheel processor.
 */
typedef struct {
    alps_scroll_state_t state;     /**< Current scroll engagement state */
    alps_axis_lock_t axis_lock;    /**< Current primary axis lock status */
    uint64_t press_time;           /**< Timestamp (ms) when middle button was pressed */
    int motion_total;              /**< Total absolute motion counts accumulated during current hold */
    int32_t acc_v_hires;           /**< Remainder accumulator for vertical wheel (units modulo 120) */
    int32_t acc_h_hires;           /**< Remainder accumulator for horizontal wheel (units modulo 120) */
    int32_t cross_axis_motion;     /**< Motion counts on orthogonal axis since primary lock */
} alps_scroll_wheel_t;

/**
 * @brief Initializes a scroll wheel context to its clean resting state.
 *
 * @param ctx Pointer to scroll wheel context.
 */
void alps_scroll_wheel_init(alps_scroll_wheel_t *ctx);

/**
 * @brief Resets the scroll wheel state machine to `ALPS_SCROLL_STATE_IDLE`.
 *
 * @param ctx Pointer to scroll wheel context.
 */
void alps_scroll_wheel_reset(alps_scroll_wheel_t *ctx);

/**
 * @brief Returns non-zero if the scroll wheel is currently engaged (`BTN_MIDDLE` held).
 *
 * @param ctx Pointer to scroll wheel context.
 * @return `1` if engaged, `0` if idle.
 */
int alps_scroll_wheel_is_engaged(const alps_scroll_wheel_t *ctx);

/**
 * @brief Feeds a normalized sensor event into the scroll wheel state machine.
 *
 * When middle-button hold is active:
 * - Vertical deflection ($-dy$) generates positive vertical wheel (scroll up).
 * - Vertical deflection ($+dy$) generates negative vertical wheel (scroll down).
 * - Horizontal deflection ($+dx$) generates positive horizontal wheel (scroll right).
 * - Horizontal deflection ($-dx$) generates negative horizontal wheel (scroll left).
 * - If `counter_drift_enabled` is true, attaches `drift_x = -dx` and `drift_y = -dy`
 *   to counter the kernel's native cursor movement in the same frame.
 *
 * @param ctx     Pointer to scroll wheel context.
 * @param cfg     Pointer to active scroll configuration.
 * @param ev      Pointer to normalized input event.
 * @param verbose Non-zero to log scroll diagnostics to `stdout`.
 * @return `ALPS_ACTION_SCROLL` if scrolling occurred, otherwise `ALPS_ACTION_NONE`.
 */
alps_action_t alps_scroll_wheel_process_event(alps_scroll_wheel_t *ctx,
                                              const alps_scroll_config_t *cfg,
                                              const alps_event_t *ev,
                                              int verbose);

#endif /* ALPS_FEATURES_SCROLL_WHEEL_H */
