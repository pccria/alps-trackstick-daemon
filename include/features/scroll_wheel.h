/**
 * @file scroll_wheel.h
 * @brief High-resolution 2D kinetic trackstick scroll wheel state machine.
 *
 * Implements a pure, zero-I/O finite state machine that intercepts physical
 * middle-button hold events (`BTN_MIDDLE`) and translates relative stick deflections
 * into continuous high-resolution vertical (`REL_WHEEL_HI_RES`) and horizontal
 * (`REL_HWHEEL_HI_RES`) scroll actions (`alps_action_t`).
 *
 * Employs a dynamic **Continuous Transient Vector Cone**:
 * - No sticky state latching: transitions between vertical, horizontal, and diagonal
 *   are instantaneous and fluid at every frame.
 * - Angular deadband: dampens lateral tremor during vertical reading and dampens
 *   vertical tremor during horizontal panning without accumulating into burst overscroll.
 * - Frame-based evaluation: aggregates multi-axis deltas per hardware `SYN_REPORT`
 *   boundary to evaluate the true physical deflection vector.
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
 * @brief Complete state context for the 2D scroll wheel processor.
 */
typedef struct {
    alps_scroll_state_t state;     /**< Current scroll engagement state */
    uint64_t press_time;           /**< Timestamp (ms) when middle button was pressed */
    int motion_total;              /**< Total absolute motion counts accumulated during current hold */
    int32_t pending_dx;            /**< Aggregated physical X deflection in current hardware frame */
    int32_t pending_dy;            /**< Aggregated physical Y deflection in current hardware frame */
    int32_t acc_v_hires;           /**< Remainder accumulator for vertical wheel (units modulo 120) */
    int32_t acc_h_hires;           /**< Remainder accumulator for horizontal wheel (units modulo 120) */
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
 * @brief Direct vector frame processor: translates instantaneous `(dx, dy)` deltas
 *        into a high-resolution 2D scroll action with vector-cone filtering.
 *
 * @param ctx     Pointer to scroll wheel context.
 * @param cfg     Pointer to active scroll configuration.
 * @param dx      Signed relative horizontal deflection in current frame.
 * @param dy      Signed relative vertical deflection in current frame.
 * @param verbose Non-zero to log scroll diagnostics to `stdout`.
 * @return `ALPS_ACTION_SCROLL` if scrolling occurred, otherwise `ALPS_ACTION_NONE`.
 */
alps_action_t alps_scroll_wheel_process_frame(alps_scroll_wheel_t *ctx,
                                              const alps_scroll_config_t *cfg,
                                              int32_t dx,
                                              int32_t dy,
                                              int verbose);

/**
 * @brief Feeds a normalized sensor event into the scroll wheel state machine.
 *
 * Aggregates `ALPS_EV_REL_X` and `ALPS_EV_REL_Y` within the current hardware frame,
 * evaluating the vector cone and emitting the scroll action upon `ALPS_EV_SYN_REPORT`
 * or standalone motion.
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
