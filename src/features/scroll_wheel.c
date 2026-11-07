/**
 * @file scroll_wheel.c
 * @brief Implementation of the 2D kinetic trackstick scroll wheel state machine with progressive ballistics.
 */

#include "features/scroll_wheel.h"
#include <stdio.h>
#include <string.h>

void alps_scroll_wheel_init(alps_scroll_wheel_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = ALPS_SCROLL_STATE_IDLE;
}

void alps_scroll_wheel_reset(alps_scroll_wheel_t *ctx) {
    alps_scroll_wheel_init(ctx);
}

int alps_scroll_wheel_is_engaged(const alps_scroll_wheel_t *ctx) {
    return (ctx && ctx->state == ALPS_SCROLL_STATE_ENGAGED) ? 1 : 0;
}

/**
 * @brief Computes high-resolution wheel ticks from directional deflection using
 *        a dual-range progressive ballistics curve.
 *
 * Curve behavior:
 * - Low deflection (|step| <= 2): Linear scaling at base_mult units/count for
 *   sub-notch reading precision (12-24 units = 10-20% of a notch).
 * - Moderate-to-firm deflection (|step| >= 3): Progressive quadratic boost
 *   allowing effortless high-speed navigation across thousands of lines.
 *
 * @param step         Signed deflection counts along an axis.
 * @param base_mult    Base multiplier for low-deflection micro-stepping.
 * @param progressive  Non-zero to enable progressive non-linear acceleration.
 * @param accel_factor Quadratic acceleration coefficient.
 * @return Signed high-resolution wheel units (120 units = 1 detent notch).
 */
static int32_t compute_ballistics(int32_t step, int base_mult, int progressive, int accel_factor) {
    if (step == 0) return 0;

    int32_t sign = step < 0 ? -1 : 1;
    int32_t mag = step < 0 ? -step : step;

    if (mag > 100) mag = 100;

    int32_t hires = mag * base_mult;

    if (progressive && mag > 2 && accel_factor > 0) {
        int32_t excess = mag - 2;
        int32_t boost = (excess * excess * accel_factor) / 4;
        hires += boost;
    }

    /* Saturation ceiling: max 1200 units (10 full notches) per frame */
    if (hires > 1200) {
        hires = 1200;
    }

    return sign * hires;
}

alps_action_t alps_scroll_wheel_process_frame(alps_scroll_wheel_t *ctx,
                                              const alps_scroll_config_t *cfg,
                                              int32_t dx,
                                              int32_t dy,
                                              int verbose) {
    alps_action_t act = alps_action_none();
    if (!ctx || !cfg || !cfg->enabled || ctx->state != ALPS_SCROLL_STATE_ENGAGED) {
        return act;
    }

    if (dx == 0 && dy == 0) {
        return act;
    }

    int32_t abs_x = dx < 0 ? -dx : dx;
    int32_t abs_y = dy < 0 ? -dy : dy;
    ctx->motion_total += (abs_x + abs_y);

    int32_t eff_x = dx;
    int32_t eff_y = dy;

    /* Continuous Transient Vector Cone Filtering:
     * Evaluated instantaneously per frame without sticky latching. */
    if (cfg->axis_lock_enabled && cfg->dominance_ratio > 0) {
        if (abs_y >= cfg->dominance_ratio * abs_x) {
            /* Predominantly vertical: dampen lateral tremor */
            eff_x = 0;
        } else if (abs_x >= cfg->dominance_ratio * abs_y) {
            /* Predominantly horizontal: dampen vertical tremor */
            eff_y = 0;
        }
        /* Else: true diagonal panning (both eff_x and eff_y retained) */
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

    /* Counter-drift: anchor pointer with exact physical inverse */
    if (cfg->counter_drift_enabled) {
        act.drift_x = -dx;
        act.drift_y = -dy;
    }

    /* Continuous High-Resolution Wheel with Progressive Ballistics (120 units = 1 notch) */
    act.wheel_v_hires = compute_ballistics(v_step, cfg->hires_multiplier,
                                           cfg->progressive_enabled, cfg->accel_factor);
    act.wheel_h_hires = compute_ballistics(h_step, cfg->hires_multiplier,
                                           cfg->progressive_enabled, cfg->accel_factor);

    /* Discrete Legacy Detents */
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

    return act;
}

alps_action_t alps_scroll_wheel_process_event(alps_scroll_wheel_t *ctx,
                                              const alps_scroll_config_t *cfg,
                                              const alps_event_t *ev,
                                              int verbose) {
    alps_action_t act = alps_action_none();
    if (!ctx || !cfg || !ev || !cfg->enabled) {
        return act;
    }

    /* 1. Middle button press / release */
    if (ev->type == ALPS_EV_BTN_MIDDLE) {
        if (ev->value == 1) {
            ctx->state = ALPS_SCROLL_STATE_ENGAGED;
            ctx->press_time = ev->timestamp_ms;
            ctx->motion_total = 0;
            ctx->pending_dx = 0;
            ctx->pending_dy = 0;
            ctx->acc_v_hires = 0;
            ctx->acc_h_hires = 0;
            if (verbose) {
                printf("  [SCROLL ENGAGED: BTN_MIDDLE DOWN]\n");
            }
        } else if (ev->value == 0) {
            /* If any motion was pending before button release, flush it */
            if (ctx->pending_dx != 0 || ctx->pending_dy != 0) {
                act = alps_scroll_wheel_process_frame(ctx, cfg, ctx->pending_dx, ctx->pending_dy, verbose);
                ctx->pending_dx = 0;
                ctx->pending_dy = 0;
            }
            ctx->state = ALPS_SCROLL_STATE_IDLE;
            if (verbose) {
                printf("  [SCROLL RELEASED: BTN_MIDDLE UP (total motion=%d counts)]\n",
                       ctx->motion_total);
            }
        }
        return act;
    }

    if (ctx->state != ALPS_SCROLL_STATE_ENGAGED) {
        return act;
    }

    /* 2. Buffer relative motion within frame */
    if (ev->type == ALPS_EV_REL_X) {
        ctx->pending_dx += ev->value;
    } else if (ev->type == ALPS_EV_REL_Y) {
        ctx->pending_dy += ev->value;
    }

    /* 3. Flush on hardware frame synchronization */
    if (ev->type == ALPS_EV_SYN_REPORT) {
        if (ctx->pending_dx != 0 || ctx->pending_dy != 0) {
            act = alps_scroll_wheel_process_frame(ctx, cfg, ctx->pending_dx, ctx->pending_dy, verbose);
            ctx->pending_dx = 0;
            ctx->pending_dy = 0;
        }
    }

    return act;
}
