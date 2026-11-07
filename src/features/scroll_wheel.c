/**
 * @file scroll_wheel.c
 * @brief Implementation of the 2D kinetic trackstick scroll wheel state machine.
 */

#include "features/scroll_wheel.h"
#include <stdio.h>
#include <string.h>

void alps_scroll_wheel_init(alps_scroll_wheel_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = ALPS_SCROLL_STATE_IDLE;
    ctx->axis_lock = ALPS_AXIS_LOCK_NONE;
}

void alps_scroll_wheel_reset(alps_scroll_wheel_t *ctx) {
    alps_scroll_wheel_init(ctx);
}

int alps_scroll_wheel_is_engaged(const alps_scroll_wheel_t *ctx) {
    return (ctx && ctx->state == ALPS_SCROLL_STATE_ENGAGED) ? 1 : 0;
}

alps_action_t alps_scroll_wheel_process_event(alps_scroll_wheel_t *ctx,
                                              const alps_scroll_config_t *cfg,
                                              const alps_event_t *ev,
                                              int verbose) {
    alps_action_t act = alps_action_none();
    if (!ctx || !cfg || !ev || !cfg->enabled) {
        return act;
    }

    /* 1. Physical middle button toggle */
    if (ev->type == ALPS_EV_BTN_MIDDLE) {
        if (ev->value == 1) {
            ctx->state = ALPS_SCROLL_STATE_ENGAGED;
            ctx->axis_lock = ALPS_AXIS_LOCK_NONE;
            ctx->press_time = ev->timestamp_ms;
            ctx->motion_total = 0;
            ctx->acc_v_hires = 0;
            ctx->acc_h_hires = 0;
            ctx->cross_axis_motion = 0;
            if (verbose) {
                printf("  [SCROLL ENGAGED: BTN_MIDDLE DOWN]\n");
            }
        } else if (ev->value == 0) {
            ctx->state = ALPS_SCROLL_STATE_IDLE;
            ctx->axis_lock = ALPS_AXIS_LOCK_NONE;
            if (verbose) {
                printf("  [SCROLL RELEASED: BTN_MIDDLE UP (total motion=%d counts)]\n",
                       ctx->motion_total);
            }
        }
        return act;
    }

    /* 2. Relative motion deflection while engaged */
    if (ctx->state == ALPS_SCROLL_STATE_ENGAGED &&
        (ev->type == ALPS_EV_REL_X || ev->type == ALPS_EV_REL_Y)) {

        int32_t raw_x = (ev->type == ALPS_EV_REL_X) ? ev->value : 0;
        int32_t raw_y = (ev->type == ALPS_EV_REL_Y) ? ev->value : 0;
        int32_t eff_x = raw_x;
        int32_t eff_y = raw_y;

        int32_t mag = ev->value < 0 ? -ev->value : ev->value;
        ctx->motion_total += mag;

        /* Primary Axis-Lock Filter: prevents lateral drift during fast reading */
        if (cfg->axis_lock_enabled) {
            if (ctx->axis_lock == ALPS_AXIS_LOCK_NONE) {
                if (raw_y != 0) {
                    ctx->axis_lock = ALPS_AXIS_LOCK_VERTICAL;
                    ctx->cross_axis_motion = 0;
                } else if (raw_x != 0) {
                    ctx->axis_lock = ALPS_AXIS_LOCK_HORIZONTAL;
                    ctx->cross_axis_motion = 0;
                }
            } else if (ctx->axis_lock == ALPS_AXIS_LOCK_VERTICAL) {
                if (raw_x != 0) {
                    ctx->cross_axis_motion += (raw_x < 0 ? -raw_x : raw_x);
                    if (ctx->cross_axis_motion >= cfg->deadband_counts) {
                        /* User intentionally steered sideways; unlock full 2D diagonal */
                        ctx->axis_lock = ALPS_AXIS_LOCK_NONE;
                    } else {
                        /* Suppress incidental horizontal tremor */
                        eff_x = 0;
                    }
                }
            } else if (ctx->axis_lock == ALPS_AXIS_LOCK_HORIZONTAL) {
                if (raw_y != 0) {
                    ctx->cross_axis_motion += (raw_y < 0 ? -raw_y : raw_y);
                    if (ctx->cross_axis_motion >= cfg->deadband_counts) {
                        /* User intentionally steered vertically; unlock full 2D diagonal */
                        ctx->axis_lock = ALPS_AXIS_LOCK_NONE;
                    } else {
                        /* Suppress incidental vertical tremor */
                        eff_y = 0;
                    }
                }
            }
        }

        /* Direction mapping:
         * Pushing stick UP (-dy) -> scroll UP (+wheel_v)
         * Pulling stick DOWN (+dy) -> scroll DOWN (-wheel_v)
         * Pushing stick RIGHT (+dx) -> scroll RIGHT (+wheel_h)
         * Pushing stick LEFT (-dx) -> scroll LEFT (-wheel_h)
         */
        int32_t v_step = -eff_y;
        int32_t h_step = eff_x;

        if (v_step == 0 && h_step == 0 && !cfg->counter_drift_enabled) {
            return act;
        }

        act.type = ALPS_ACTION_SCROLL;

        /* Counter-drift: snap cursor back by (-raw_x, -raw_y) in same frame */
        if (cfg->counter_drift_enabled) {
            act.drift_x = -raw_x;
            act.drift_y = -raw_y;
        }

        /* Continuous High-Resolution Wheel (120 units = 1 notch) */
        act.wheel_v_hires = v_step * cfg->hires_multiplier;
        act.wheel_h_hires = h_step * cfg->hires_multiplier;

        /* Discrete Legacy Detents (emitted when accumulated units cross +/-120) */
        ctx->acc_v_hires += act.wheel_v_hires;
        ctx->acc_h_hires += act.wheel_h_hires;

        while (ctx->acc_v_hires >= 120) {
            act.wheel_v += 1;
            ctx->acc_v_hires -= 120;
        }
        while (ctx->acc_v_hires <= -120) {
            act.wheel_v -= 1;
            ctx->acc_v_hires += 120;
        }
        while (ctx->acc_h_hires >= 120) {
            act.wheel_h += 1;
            ctx->acc_h_hires -= 120;
        }
        while (ctx->acc_h_hires <= -120) {
            act.wheel_h -= 1;
            ctx->acc_h_hires += 120;
        }

        if (verbose && (act.wheel_v_hires != 0 || act.wheel_h_hires != 0)) {
            printf("  [SCROLL 2D: hires=[%+d, %+d], detent=[%+d, %+d], drift=[%+d, %+d]]\n",
                   act.wheel_v_hires, act.wheel_h_hires,
                   act.wheel_v, act.wheel_h,
                   act.drift_x, act.drift_y);
        }
    }

    return act;
}
