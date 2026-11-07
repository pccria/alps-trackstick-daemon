/**
 * @file scroll_wheel.c
 * @brief Implementation of the 2D kinetic trackstick scroll wheel state machine with progressive ballistics
 *        and continuous C^1 Hermite angle blending.
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

/**
 * @brief Computes continuous Hermite smoothstep weight [0..256] for cross-axis blending.
 *
 * Implements S(t) = 3t^2 - 2t^3 in 8-bit fixed-point arithmetic:
 * - When cross/dominant <= 1/3 (angle within ~18 deg of primary axis):
 *   weight = 0 (100% tremor suppression, preventing accidental browser navigation).
 * - When cross/dominant >= 2/3 (angle within ~34 deg of diagonal):
 *   weight = 256 (100% unconstrained 2D diagonal transmission).
 * - Intermediate zone (1/3 < cross/dominant < 2/3):
 *   C^1 mathematically continuous Hermite ramp without step-discontinuities.
 *
 * @param cross     Cross-axis absolute magnitude (|dx| or |dy|).
 * @param dominant  Dominant-axis absolute magnitude (must be >= cross).
 * @return Fixed-point weight in range [0..256] where 256 = 1.0 (100% transmission).
 */
static int32_t compute_smooth_weight_fp(int32_t cross, int32_t dominant) {
    if (dominant <= 0) return 256;

    /* Lower deadband threshold: 3 * cross <= dominant -> weight = 0 */
    if (3 * cross <= dominant) {
        return 0;
    }

    /* Upper full-transmission threshold: 3 * cross >= 2 * dominant -> weight = 256 */
    if (3 * cross >= 2 * dominant) {
        return 256;
    }

    /* Normalized parameter t in [0..256] = 256 * (3*cross - dominant) / dominant */
    int32_t num = 3 * cross - dominant;
    int32_t t_fp = (num * 256) / dominant;
    if (t_fp < 0) t_fp = 0;
    if (t_fp > 256) t_fp = 256;

    /* Fixed-point cubic Hermite smoothstep: S(t) = t^2 * (3 - 2t)
     * t2 scale is 2^8, term scale is 2^8, product scale is 2^16.
     * Shift right by 8 to yield result at 2^8 (scale 256). */
    int32_t t2 = (t_fp * t_fp) >> 8;
    int32_t term = (3 * 256) - (2 * t_fp);
    int32_t s_fp = (t2 * term) >> 8;

    if (s_fp < 0) s_fp = 0;
    if (s_fp > 256) s_fp = 256;
    return s_fp;
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

    /* Continuous C^1 Trigonometric Vector Cone Filtering:
     * Eliminates step-discontinuities when curving mid-drag between axes. */
    if (cfg->axis_lock_enabled) {
        if (abs_y >= abs_x) {
            /* Vertical dominant: smoothly filter lateral component */
            int32_t weight = compute_smooth_weight_fp(abs_x, abs_y);
            eff_x = (dx * weight + (dx >= 0 ? 128 : -128)) / 256;
        } else {
            /* Horizontal dominant: smoothly filter vertical component */
            int32_t weight = compute_smooth_weight_fp(abs_y, abs_x);
            eff_y = (dy * weight + (dy >= 0 ? 128 : -128)) / 256;
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

    if (v_step == 0 && h_step == 0) {
        return act;
    }

    act.type = ALPS_ACTION_SCROLL;

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
        printf("  [SCROLL 2D: hires=[%+d, %+d], detent=[%+d, %+d]]\n",
               act.wheel_v_hires, act.wheel_h_hires,
               act.wheel_v, act.wheel_h);
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
