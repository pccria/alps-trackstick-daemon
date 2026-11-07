/**
 * @file tap_gesture.c
 * @brief Implementation of the anti-drift physical transducer state machine.
 *
 * Physical Principles Modeled:
 * 1. **Zero-Latency Rebound (`ALPS_TAP_STATE_TAP2_DOWN`)**:
 *    Emits `ALPS_ACTION_BUTTON_DOWN` on the exact millisecond `ABS_PRESSURE`
 *    crosses `tap2_rebound_pressure` (15), guaranteeing double-clicks land
 *    inside strict OS timing windows and eliminating post-release silence loops.
 *
 * 2. **Force-Ratio Discriminator**:
 *    Distinguishes intentional light vertical taps (`Z >= 22`) from lateral
 *    steering micro-nudges by requiring `max_pressure >= (tap1_motion * 1.5)`
 *    whenever contact shear exceeds 6 counts.
 *
 * 3. **Hardware Anti-Drift Snap**:
 *    Tracks net signed cursor displacement (`tap1_dx`, `tap1_dy`) during the
 *    brief contact window and attaches the inverse vector (`-dx`, `-dy`) to
 *    the button press/release action so the cursor snaps back to the exact pre-tap pixel.
 *
 * 4. **100% Non-Blocking Zero-`usleep` Architecture**:
 *    Button hold durations (`click_hold_ms`) are managed strictly through state
 *    deadlines (`ALPS_TAP_STATE_TAP1_HOLD` and `ALPS_TAP_STATE_TAP2_HOLD`) and
 *    `epoll_wait` timeouts. The event loop never stalls or sleeps.
 *
 * 5. **Lateral Shear Drag Retention (`ALPS_TAP_STATE_DRAGGING`)**:
 *    Pushing the trackstick sideways unloads vertical strain (`ABS_PRESSURE -> 0`).
 *    Active drags are therefore sustained by incoming `EV_REL` steering packets
 *    with a configurable inactivity grace window (`drag_release_grace_ms`).
 */

#include "features/tap_gesture.h"
#include <stdio.h>
#include <string.h>

void alps_tap_gesture_init(alps_tap_gesture_t *ctx) {
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = ALPS_TAP_STATE_IDLE;
}

alps_action_t alps_tap_gesture_reset(alps_tap_gesture_t *ctx) {
    alps_action_t act = alps_action_none();
    if (!ctx) return act;

    if (ctx->state == ALPS_TAP_STATE_TAP1_HOLD ||
        ctx->state == ALPS_TAP_STATE_TAP2_DOWN ||
        ctx->state == ALPS_TAP_STATE_TAP2_HOLD ||
        ctx->state == ALPS_TAP_STATE_DRAGGING) {
        act.type = ALPS_ACTION_BUTTON_UP;
    }

    alps_tap_gesture_init(ctx);
    return act;
}

int alps_tap_gesture_get_timeout_ms(const alps_tap_gesture_t *ctx,
                                    const alps_tap_config_t *cfg,
                                    uint64_t now_ms) {
    if (!ctx || !cfg) return -1;

    switch (ctx->state) {
        case ALPS_TAP_STATE_TAP1_DOWN: {
            uint64_t limit = (uint64_t)cfg->max_tap_duration_ms + 50ULL;
            uint64_t held = now_ms >= ctx->press_time ? (now_ms - ctx->press_time) : 0;
            return (held >= limit) ? 0 : (int)(limit - held);
        }
        case ALPS_TAP_STATE_TAP1_HOLD: {
            return (now_ms >= ctx->click_end_time) ? 0 : (int)(ctx->click_end_time - now_ms);
        }
        case ALPS_TAP_STATE_AWAITING_TAP2: {
            uint64_t limit = (uint64_t)cfg->double_tap_window_ms;
            uint64_t elapsed = now_ms >= ctx->tap1_release_time ? (now_ms - ctx->tap1_release_time) : 0;
            return (elapsed >= limit) ? 0 : (int)(limit - elapsed);
        }
        case ALPS_TAP_STATE_TAP2_DOWN: {
            uint64_t limit = (uint64_t)cfg->drag_start_hold_ms;
            uint64_t held = now_ms >= ctx->press_time ? (now_ms - ctx->press_time) : 0;
            return (held >= limit) ? 0 : (int)(limit - held);
        }
        case ALPS_TAP_STATE_TAP2_HOLD: {
            return (now_ms >= ctx->click_end_time) ? 0 : (int)(ctx->click_end_time - now_ms);
        }
        case ALPS_TAP_STATE_DRAGGING: {
            uint64_t limit = (uint64_t)cfg->drag_release_grace_ms;
            uint64_t inactive = now_ms >= ctx->last_active_time ? (now_ms - ctx->last_active_time) : 0;
            return (inactive >= limit) ? 0 : (int)(limit - inactive);
        }
        case ALPS_TAP_STATE_IDLE:
        default:
            return -1;
    }
}

alps_action_t alps_tap_gesture_handle_timeout(alps_tap_gesture_t *ctx,
                                              const alps_tap_config_t *cfg,
                                              uint64_t now_ms,
                                              int verbose) {
    alps_action_t act = alps_action_none();
    if (!ctx || !cfg) return act;

    switch (ctx->state) {
        case ALPS_TAP_STATE_TAP1_DOWN: {
            uint64_t held = now_ms >= ctx->press_time ? (now_ms - ctx->press_time) : 0;
            if (held >= (uint64_t)cfg->max_tap_duration_ms + 50ULL) {
                ctx->state = ALPS_TAP_STATE_IDLE;
            }
            break;
        }
        case ALPS_TAP_STATE_TAP1_HOLD: {
            if (now_ms >= ctx->click_end_time) {
                act.type = ALPS_ACTION_BUTTON_UP;
                ctx->tap1_release_time = now_ms;
                if (ctx->await_motion >= cfg->await_motion_cancel) {
                    ctx->state = ALPS_TAP_STATE_IDLE;
                    if (verbose) {
                        printf("  [AWAIT CANCELLED: Navigation Steering during hold (%d counts)]\n",
                               ctx->await_motion);
                    }
                } else {
                    ctx->state = ALPS_TAP_STATE_AWAITING_TAP2;
                    if (verbose) {
                        printf("-> TAP 1 CLICK RELEASED (hold complete, awaiting tap 2)\n");
                    }
                }
            }
            break;
        }
        case ALPS_TAP_STATE_AWAITING_TAP2: {
            uint64_t elapsed = now_ms >= ctx->tap1_release_time ? (now_ms - ctx->tap1_release_time) : 0;
            if (elapsed >= (uint64_t)cfg->double_tap_window_ms) {
                ctx->state = ALPS_TAP_STATE_IDLE;
            }
            break;
        }
        case ALPS_TAP_STATE_TAP2_DOWN: {
            uint64_t held = now_ms >= ctx->press_time ? (now_ms - ctx->press_time) : 0;
            if (held >= (uint64_t)cfg->drag_start_hold_ms) {
                ctx->state = ALPS_TAP_STATE_DRAGGING;
                ctx->last_active_time = now_ms;
                if (verbose) {
                    printf("  [DRAG ENGAGED: Stationary Hold (%lums)]\n", (unsigned long)held);
                }
            }
            break;
        }
        case ALPS_TAP_STATE_TAP2_HOLD: {
            if (now_ms >= ctx->click_end_time) {
                act.type = ALPS_ACTION_BUTTON_UP;
                if (cfg->anti_drift_enabled) {
                    act.drift_x = -ctx->tap2_dx;
                    act.drift_y = -ctx->tap2_dy;
                }
                ctx->state = ALPS_TAP_STATE_IDLE;
                if (verbose) {
                    printf("-> DOUBLE CLICK COMPLETED (Tap 2 hold complete, drift=[%+d, %+d])\n",
                           ctx->tap2_dx, ctx->tap2_dy);
                }
            }
            break;
        }
        case ALPS_TAP_STATE_DRAGGING: {
            uint64_t inactive = now_ms >= ctx->last_active_time ? (now_ms - ctx->last_active_time) : 0;
            if (inactive >= (uint64_t)cfg->drag_release_grace_ms) {
                act.type = ALPS_ACTION_BUTTON_UP;
                ctx->state = ALPS_TAP_STATE_IDLE;
                if (verbose) {
                    printf("  [DRAG RELEASED: Inactivity Grace (%ums)]\n", cfg->drag_release_grace_ms);
                }
            }
            break;
        }
        case ALPS_TAP_STATE_IDLE:
        default:
            break;
    }

    return act;
}

/**
 * @brief Internal helper to process relative motion events (`REL_X` / `REL_Y`).
 */
static alps_action_t process_relative_motion(alps_tap_gesture_t *ctx,
                                             const alps_tap_config_t *cfg,
                                             const alps_event_t *ev,
                                             int verbose) {
    alps_action_t act = alps_action_none();
    uint64_t now = ev->timestamp_ms;

    /* Update decaying pre-press navigation velocity tracker */
    uint64_t rel_dt = now >= ctx->last_rel_time ? (now - ctx->last_rel_time) : 0;
    if (rel_dt > (uint64_t)cfg->pre_motion_window_ms) {
        ctx->pre_motion = 0;
    } else if (rel_dt > 60ULL) {
        ctx->pre_motion /= 2;
    }
    ctx->pre_motion = alps_add_motion_saturated(ctx->pre_motion, ev->value, cfg->motion_cap);
    ctx->last_rel_time = now;

    switch (ctx->state) {
        case ALPS_TAP_STATE_TAP1_DOWN:
            ctx->tap1_motion = alps_add_motion_saturated(ctx->tap1_motion, ev->value, cfg->motion_cap);
            if (ev->type == ALPS_EV_REL_X) {
                ctx->tap1_dx += ev->value;
            } else if (ev->type == ALPS_EV_REL_Y) {
                ctx->tap1_dy += ev->value;
            }
            break;

        case ALPS_TAP_STATE_TAP1_HOLD:
        case ALPS_TAP_STATE_AWAITING_TAP2:
            ctx->await_motion = alps_add_motion_saturated(ctx->await_motion, ev->value, cfg->motion_cap);
            if (ctx->state == ALPS_TAP_STATE_AWAITING_TAP2 && ctx->await_motion >= cfg->await_motion_cancel) {
                /* Active pointer steering: cancel primed double-tap window immediately */
                ctx->state = ALPS_TAP_STATE_IDLE;
                if (verbose) {
                    printf("  [AWAIT CANCELLED: Navigation Steering (%d counts)]\n", ctx->await_motion);
                }
            }
            break;

        case ALPS_TAP_STATE_TAP2_DOWN:
            ctx->tap2_motion = alps_add_motion_saturated(ctx->tap2_motion, ev->value, cfg->motion_cap);
            if (ev->type == ALPS_EV_REL_X) {
                ctx->tap2_dx += ev->value;
            } else if (ev->type == ALPS_EV_REL_Y) {
                ctx->tap2_dy += ev->value;
            }
            if (ctx->tap2_motion >= cfg->drag_start_motion) {
                ctx->state = ALPS_TAP_STATE_DRAGGING;
                ctx->last_active_time = now;
                if (verbose) {
                    printf("  [DRAG ENGAGED: Instant Steering Lock (%d counts)]\n", ctx->tap2_motion);
                }
            }
            break;

        case ALPS_TAP_STATE_TAP2_HOLD:
            /* Button-up timer is running; allow brief motion post-liftoff */
            break;

        case ALPS_TAP_STATE_DRAGGING:
            ctx->last_active_time = now;
            break;

        case ALPS_TAP_STATE_IDLE:
        default:
            break;
    }

    return act;
}

/**
 * @brief Internal helper to process Z-axis pressure events (`ABS_PRESSURE`).
 */
static alps_action_t process_pressure(alps_tap_gesture_t *ctx,
                                      const alps_tap_config_t *cfg,
                                      const alps_event_t *ev,
                                      int verbose) {
    alps_action_t act = alps_action_none();
    int val = ev->value;
    uint64_t now = ev->timestamp_ms;

    /* 1. Touchdown / Active Contact Phase */
    if (ctx->state == ALPS_TAP_STATE_IDLE) {
        if (val >= cfg->tap1_pressure) {
            uint64_t rel_elapsed = now >= ctx->last_rel_time ? (now - ctx->last_rel_time) : 0;
            if (rel_elapsed <= (uint64_t)cfg->pre_motion_window_ms &&
                ctx->pre_motion >= cfg->pre_motion_limit) {
                if (verbose) {
                    printf("  [TAP 1 IGNORED: Steering Velocity (pre_motion=%d, Z=%d)]\n",
                           ctx->pre_motion, val);
                }
            } else {
                ctx->state = ALPS_TAP_STATE_TAP1_DOWN;
                ctx->press_time = now;
                ctx->max_pressure = val;
                ctx->tap1_motion = 0;
                ctx->tap1_dx = 0;
                ctx->tap1_dy = 0;
            }
        }
    } else if (ctx->state == ALPS_TAP_STATE_TAP1_DOWN) {
        if (val > ctx->max_pressure) {
            ctx->max_pressure = val;
        }
    } else if (ctx->state == ALPS_TAP_STATE_TAP1_HOLD || ctx->state == ALPS_TAP_STATE_AWAITING_TAP2) {
        if (val >= cfg->tap2_rebound_pressure) {
            int was_holding = (ctx->state == ALPS_TAP_STATE_TAP1_HOLD);
            ctx->state = ALPS_TAP_STATE_TAP2_DOWN;
            ctx->press_time = now;
            ctx->tap2_motion = 0;
            ctx->tap2_dx = 0;
            ctx->tap2_dy = 0;
            if (!was_holding) {
                /* Button was released; emit immediate virtual button down on second strike */
                act.type = ALPS_ACTION_BUTTON_DOWN;
            }
            if (verbose) {
                printf("  [TAP 2 CONTACT: Pressure=%d -> %s]\n",
                       val, was_holding ? "Sustaining BTN_LEFT DOWN" : "BTN_LEFT DOWN");
            }
            return act;
        }
    } else if (ctx->state == ALPS_TAP_STATE_DRAGGING) {
        if (val > cfg->release_pressure) {
            ctx->last_active_time = now;
        }
    }

    /* 2. Lift-Off / Pressure Unload Phase */
    if (val <= cfg->release_pressure) {
        if (ctx->state == ALPS_TAP_STATE_TAP1_DOWN) {
            uint64_t duration = now >= ctx->press_time ? (now - ctx->press_time) : 0;

            /* Physical Discriminator: brief impulse, bounded shear, and vertical force dominance */
            int is_valid_tap = (duration <= (uint64_t)cfg->max_tap_duration_ms) &&
                               (ctx->tap1_motion <= cfg->max_tap_motion) &&
                               (ctx->tap1_motion <= 6 || ctx->max_pressure >= (ctx->tap1_motion * 3 / 2));

            if (is_valid_tap) {
                /* Non-blocking: emit button down + anti-drift immediately, then enter hold state */
                act.type = ALPS_ACTION_BUTTON_DOWN;
                if (cfg->anti_drift_enabled) {
                    act.drift_x = -ctx->tap1_dx;
                    act.drift_y = -ctx->tap1_dy;
                }
                ctx->state = ALPS_TAP_STATE_TAP1_HOLD;
                ctx->click_end_time = now + (uint64_t)cfg->click_hold_ms;
                ctx->await_motion = 0;
                if (verbose) {
                    printf("-> TAP 1 PRESS (duration=%lums, motion=%d, drift=[%+d, %+d], max_Z=%d)\n",
                           (unsigned long)duration, ctx->tap1_motion,
                           ctx->tap1_dx, ctx->tap1_dy, ctx->max_pressure);
                }
            } else {
                ctx->state = ALPS_TAP_STATE_IDLE;
                if (verbose) {
                    printf("  [TAP 1 REJECTED: duration=%lums, motion=%d, max_Z=%d (ratio check)]\n",
                           (unsigned long)duration, ctx->tap1_motion, ctx->max_pressure);
                }
            }
        } else if (ctx->state == ALPS_TAP_STATE_TAP2_DOWN) {
            uint64_t duration = now >= ctx->press_time ? (now - ctx->press_time) : 0;
            if (duration <= (uint64_t)cfg->max_tap_duration_ms &&
                ctx->tap2_motion < cfg->drag_start_motion) {
                /* Discrete second tap */
                if (duration < (uint64_t)cfg->click_hold_ms) {
                    /* Hold for remainder of click_hold_ms non-blockingly via timer */
                    ctx->state = ALPS_TAP_STATE_TAP2_HOLD;
                    ctx->click_end_time = now + ((uint64_t)cfg->click_hold_ms - duration);
                } else {
                    /* Already held long enough: release immediately */
                    act.type = ALPS_ACTION_BUTTON_UP;
                    if (cfg->anti_drift_enabled) {
                        act.drift_x = -ctx->tap2_dx;
                        act.drift_y = -ctx->tap2_dy;
                    }
                    ctx->state = ALPS_TAP_STATE_IDLE;
                    if (verbose) {
                        printf("-> DOUBLE CLICK COMPLETED (Tap 2 duration=%lums, motion=%d, drift=[%+d, %+d])\n",
                               (unsigned long)duration, ctx->tap2_motion, ctx->tap2_dx, ctx->tap2_dy);
                    }
                }
            } else {
                /* Held or moved beyond threshold: lock into drag */
                ctx->state = ALPS_TAP_STATE_DRAGGING;
                ctx->last_active_time = now;
                if (verbose) {
                    printf("  [DRAG ENGAGED: Sustained Tap 2 (motion=%d)]\n", ctx->tap2_motion);
                }
            }
        }
        /* In ALPS_TAP_STATE_DRAGGING: pressure unloading to 0 is expected when pushing
         * the stick laterally; drag remains locked via EV_REL stream + grace timer. */
    }

    return act;
}

alps_action_t alps_tap_gesture_process_event(alps_tap_gesture_t *ctx,
                                             const alps_tap_config_t *cfg,
                                             const alps_event_t *ev,
                                             int verbose) {
    if (!ctx || !cfg || !ev) {
        return alps_action_none();
    }

    if (ev->type == ALPS_EV_REL_X || ev->type == ALPS_EV_REL_Y) {
        return process_relative_motion(ctx, cfg, ev, verbose);
    } else if (ev->type == ALPS_EV_PRESSURE) {
        return process_pressure(ctx, cfg, ev, verbose);
    }

    return alps_action_none();
}
