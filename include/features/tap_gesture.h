/**
 * @file tap_gesture.h
 * @brief Anti-drift physical transducer state machine for tap, double-click, and drag.
 *
 * Implements a deterministic, zero-I/O finite state machine that translates
 * strain-gauge pressure (`ABS_PRESSURE`) and relative deflection (`REL_X`/`REL_Y`)
 * into discrete virtual mouse actions (`alps_action_t`).
 */

#ifndef ALPS_FEATURES_TAP_GESTURE_H
#define ALPS_FEATURES_TAP_GESTURE_H

#include <stdint.h>
#include "core/config.h"
#include "core/event.h"
#include "core/action.h"

/**
 * @brief Internal lifecycle states of the tap/drag transducer.
 */
typedef enum {
    ALPS_TAP_STATE_IDLE = 0,       /**< Resting/navigating state, awaiting initial tap */
    ALPS_TAP_STATE_TAP1_DOWN,      /**< Finger in contact for Tap 1; measuring duration & shear */
    ALPS_TAP_STATE_AWAITING_TAP2,  /**< Tap 1 clicked; primed window open for Tap 2 or drag */
    ALPS_TAP_STATE_TAP2_DOWN,      /**< Finger in contact for Tap 2; BTN_LEFT is immediately DOWN */
    ALPS_TAP_STATE_DRAGGING        /**< Active drag lock; sustained by EV_REL stream or pressure */
} alps_tap_state_t;

/**
 * @brief Complete state context for the tap/drag gesture processor.
 */
typedef struct {
    alps_tap_state_t state;        /**< Current state machine phase */
    uint64_t press_time;           /**< Timestamp (ms) when current contact began */
    uint64_t tap1_release_time;    /**< Timestamp (ms) when Tap 1 lifted off */
    uint64_t last_active_time;     /**< Timestamp (ms) of last motion or pressure during drag */
    uint64_t last_rel_time;        /**< Timestamp (ms) of last relative motion event */
    int max_pressure;              /**< Peak Z pressure recorded during Tap 1 contact */
    int tap1_motion;               /**< Accumulated scalar motion magnitude during Tap 1 */
    int32_t tap1_dx;               /**< Net signed X drift accumulated during Tap 1 */
    int32_t tap1_dy;               /**< Net signed Y drift accumulated during Tap 1 */
    int tap2_motion;               /**< Accumulated scalar motion magnitude during Tap 2 */
    int32_t tap2_dx;               /**< Net signed X drift accumulated during Tap 2 */
    int32_t tap2_dy;               /**< Net signed Y drift accumulated during Tap 2 */
    int await_motion;              /**< Motion accumulated while in AWAITING_TAP2 */
    int pre_motion;                /**< Decaying pre-press navigation velocity accumulator */
} alps_tap_gesture_t;

/**
 * @brief Initializes a tap gesture context to its clean resting state.
 *
 * @param ctx Pointer to gesture context.
 */
void alps_tap_gesture_init(alps_tap_gesture_t *ctx);

/**
 * @brief Resets the state machine to `ALPS_TAP_STATE_IDLE` and outputs a release
 *        action if the virtual button was held down.
 *
 * Call this during daemon shutdown or when another feature (such as a scroll
 * module) preempts pointer control, guaranteeing `BTN_LEFT` is never stuck down.
 *
 * @param ctx Pointer to gesture context.
 * @return `ALPS_ACTION_BUTTON_UP` if a button was held down, otherwise `ALPS_ACTION_NONE`.
 */
alps_action_t alps_tap_gesture_reset(alps_tap_gesture_t *ctx);

/**
 * @brief Computes the next `epoll_wait` timeout required by the state machine.
 *
 * @param ctx    Pointer to gesture context.
 * @param cfg    Pointer to active tap configuration.
 * @param now_ms Current monotonic timestamp in milliseconds.
 * @return Timeout in milliseconds (`>= 0`), or `-1` if no timer is active (`IDLE`).
 */
int alps_tap_gesture_get_timeout_ms(const alps_tap_gesture_t *ctx,
                                    const alps_tap_config_t *cfg,
                                    uint64_t now_ms);

/**
 * @brief Advances the state machine when an `epoll_wait` timer expires or before
 *        sleeping, emitting any timer-driven actions (such as drag release).
 *
 * @param ctx     Pointer to gesture context.
 * @param cfg     Pointer to active tap configuration.
 * @param now_ms  Current monotonic timestamp in milliseconds.
 * @param verbose Non-zero to log state transitions to `stdout`.
 * @return Action to dispatch (`ALPS_ACTION_BUTTON_UP` on drag expiry, else `ALPS_ACTION_NONE`).
 */
alps_action_t alps_tap_gesture_handle_timeout(alps_tap_gesture_t *ctx,
                                              const alps_tap_config_t *cfg,
                                              uint64_t now_ms,
                                              int verbose);

/**
 * @brief Feeds a normalized sensor event (`REL_X`, `REL_Y`, or `PRESSURE`) into
 *        the transducer state machine.
 *
 * @param ctx     Pointer to gesture context.
 * @param cfg     Pointer to active tap configuration.
 * @param ev      Pointer to normalized input event.
 * @param verbose Non-zero to log gesture diagnostics to `stdout`.
 * @return Action to dispatch via the `uinput` emitter.
 */
alps_action_t alps_tap_gesture_process_event(alps_tap_gesture_t *ctx,
                                             const alps_tap_config_t *cfg,
                                             const alps_event_t *ev,
                                             int verbose);

#endif /* ALPS_FEATURES_TAP_GESTURE_H */
