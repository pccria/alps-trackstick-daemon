/**
 * @file test_tap_gesture.c
 * @brief Deterministic unit test suite for the ALPS trackstick transducer pipeline.
 *
 * Validates all physical invariants of the 100% non-blocking state machine
 * without requiring root privileges, `/dev/input`, or `/dev/uinput`:
 * 1. Feather-light single tap + non-blocking hold + anti-drift snap-back (`-dx, -dy`).
 * 2. Force-ratio discriminator rejecting lateral micro-nudges (`Z < motion * 1.5`).
 * 3. Pre-motion velocity tracker rejecting high-speed steering spikes.
 * 4. Zero-latency double-click (`BUTTON_DOWN` on Tap 2 contact, non-blocking hold release).
 * 5. Ultra-fast rebound (Tap 2 contact arrives during Tap 1 hold phase).
 * 6. Immediate navigation cancel (`await_motion_cancel = 3`) after a single click.
 * 7. Tap-and-drag lock surviving `Z = 0` lateral shear and releasing after `140ms` grace.
 * 8. Saturating integer math (`INT32_MIN` and overflow protection).
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "core/config.h"
#include "core/event.h"
#include "core/action.h"
#include "features/tap_gesture.h"

static alps_event_t make_ev(alps_event_type_t type, int32_t value, uint64_t ts_ms) {
    alps_event_t ev;
    ev.type = type;
    ev.value = value;
    ev.timestamp_ms = ts_ms;
    return ev;
}

static void test_single_tap_and_anti_drift(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t ctx;
    alps_tap_gesture_init(&ctx);

    /* Touchdown at t=1000ms with Z=24 */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 24, 1000);
    alps_action_t act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_TAP1_DOWN);

    /* Slight contact shear during tap (+2 X, -1 Y) */
    ev = make_ev(ALPS_EV_REL_X, 2, 1010);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_REL_Y, -1, 1015);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);

    /* Peak pressure Z=32 */
    ev = make_ev(ALPS_EV_PRESSURE, 32, 1020);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);

    /* Liftoff at t=1045ms (Z=0): emits BUTTON_DOWN with anti-drift snap */
    ev = make_ev(ALPS_EV_PRESSURE, 0, 1045);
    act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);

    assert(act.type == ALPS_ACTION_BUTTON_DOWN);
    assert(act.drift_x == -2);
    assert(act.drift_y == 1);
    assert(ctx.state == ALPS_TAP_STATE_TAP1_HOLD);

    /* Halfway through hold at t=1055ms (10ms < 15ms hold) */
    assert(alps_tap_gesture_get_timeout_ms(&ctx, &cfg.tap, 1055) == 5);
    act = alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 1055, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_TAP1_HOLD);

    /* Hold expired at t=1060ms: emits BUTTON_UP and transitions to AWAITING_TAP2 */
    assert(alps_tap_gesture_get_timeout_ms(&ctx, &cfg.tap, 1060) == 0);
    act = alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 1060, 0);
    assert(act.type == ALPS_ACTION_BUTTON_UP);
    assert(ctx.state == ALPS_TAP_STATE_AWAITING_TAP2);

    /* Awaiting Tap 2 window expires at t=1060 + 260 = 1320ms */
    act = alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 1320, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_IDLE);

    printf("  [PASS] test_single_tap_and_anti_drift\n");
}

static void test_micro_nudge_force_ratio_rejection(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t ctx;
    alps_tap_gesture_init(&ctx);

    /* Barely cross Z=22 while pushing laterally by 16 counts (22 < 16 * 1.5 = 24) */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 22, 2000);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(ctx.state == ALPS_TAP_STATE_TAP1_DOWN);

    ev = make_ev(ALPS_EV_REL_X, 10, 2015);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_REL_Y, 6, 2025);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);

    /* Liftoff at t=2050ms */
    ev = make_ev(ALPS_EV_PRESSURE, 0, 2050);
    alps_action_t act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);

    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_IDLE);

    printf("  [PASS] test_micro_nudge_force_ratio_rejection\n");
}

static void test_high_velocity_navigation_rejection(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t ctx;
    alps_tap_gesture_init(&ctx);

    /* Active pointer navigation prior to Z spike */
    alps_event_t ev = make_ev(ALPS_EV_REL_X, 8, 3000);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_REL_Y, -6, 3020);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);

    /* Pressure spike Z=35 during steering */
    ev = make_ev(ALPS_EV_PRESSURE, 35, 3030);
    alps_action_t act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);

    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_IDLE);

    printf("  [PASS] test_high_velocity_navigation_rejection\n");
}

static void test_double_click_zero_latency_and_release(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t ctx;
    alps_tap_gesture_init(&ctx);

    /* Tap 1 */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 28, 4000);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 4040);
    alps_action_t act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_BUTTON_DOWN);
    assert(ctx.state == ALPS_TAP_STATE_TAP1_HOLD);

    /* Hold expires at 4055ms */
    act = alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 4055, 0);
    assert(act.type == ALPS_ACTION_BUTTON_UP);
    assert(ctx.state == ALPS_TAP_STATE_AWAITING_TAP2);

    /* Tap 2 touchdown at t=4110ms with gentle rebound Z=16 */
    ev = make_ev(ALPS_EV_PRESSURE, 16, 4110);
    act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_BUTTON_DOWN);
    assert(ctx.state == ALPS_TAP_STATE_TAP2_DOWN);

    /* Slight drift during Tap 2 (-1 X, +2 Y) */
    ev = make_ev(ALPS_EV_REL_X, -1, 4115);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_REL_Y, 2, 4118);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);

    /* Fast release at t=4120ms (10ms contact < 15ms hold requirement) */
    ev = make_ev(ALPS_EV_PRESSURE, 0, 4120);
    act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_TAP2_HOLD);

    /* Remaining hold expires at t=4125ms -> emits BUTTON_UP with anti-drift */
    act = alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 4125, 0);
    assert(act.type == ALPS_ACTION_BUTTON_UP);
    assert(act.drift_x == 1);
    assert(act.drift_y == -2);
    assert(ctx.state == ALPS_TAP_STATE_IDLE);

    printf("  [PASS] test_double_click_zero_latency_and_release\n");
}

static void test_ultra_fast_rebound_during_tap1_hold(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t ctx;
    alps_tap_gesture_init(&ctx);

    /* Tap 1 touchdown & liftoff */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 26, 7000);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 7020);
    alps_action_t act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_BUTTON_DOWN);
    assert(ctx.state == ALPS_TAP_STATE_TAP1_HOLD);

    /* Rapid second strike touches down at t=7025ms (only 5ms into Tap 1 hold) */
    ev = make_ev(ALPS_EV_PRESSURE, 20, 7025);
    act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    /* Button is already down, transition seamlessly to TAP2_DOWN */
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_TAP2_DOWN);

    /* User steers laterally -> locks into drag */
    ev = make_ev(ALPS_EV_REL_X, 8, 7040);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(ctx.state == ALPS_TAP_STATE_DRAGGING);

    printf("  [PASS] test_ultra_fast_rebound_during_tap1_hold\n");
}

static void test_click_then_steer_cancellation(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t ctx;
    alps_tap_gesture_init(&ctx);

    /* Tap 1 */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 25, 5000);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 5035);
    alps_action_t act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_BUTTON_DOWN);
    assert(ctx.state == ALPS_TAP_STATE_TAP1_HOLD);

    /* Hold expires */
    act = alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 5050, 0);
    assert(act.type == ALPS_ACTION_BUTTON_UP);
    assert(ctx.state == ALPS_TAP_STATE_AWAITING_TAP2);

    /* Steer 3 counts away -> immediately cancels AWAITING_TAP2 */
    ev = make_ev(ALPS_EV_REL_X, 3, 5060);
    act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_IDLE);

    printf("  [PASS] test_click_then_steer_cancellation\n");
}

static void test_tap_and_drag_across_zero_pressure_shear(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t ctx;
    alps_tap_gesture_init(&ctx);

    /* Tap 1 */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 26, 6000);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 6030);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 6045, 0);
    assert(ctx.state == ALPS_TAP_STATE_AWAITING_TAP2);

    /* Tap 2 touchdown -> immediate button down */
    ev = make_ev(ALPS_EV_PRESSURE, 18, 6090);
    alps_action_t act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_BUTTON_DOWN);

    /* Lateral steering >= 6 counts locks drag immediately */
    ev = make_ev(ALPS_EV_REL_X, 6, 6110);
    act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_DRAGGING);

    /* Lateral shear unloads vertical strain gauge to Z=0 -> drag MUST stay locked */
    ev = make_ev(ALPS_EV_PRESSURE, 0, 6120);
    act = alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_DRAGGING);

    /* Keep dragging via EV_REL up to t=6500ms */
    ev = make_ev(ALPS_EV_REL_Y, 4, 6500);
    alps_tap_gesture_process_event(&ctx, &cfg.tap, &ev, 0);
    assert(ctx.state == ALPS_TAP_STATE_DRAGGING);

    /* At t=6600ms (100ms inactive < 140ms grace), still dragging */
    act = alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 6600, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(ctx.state == ALPS_TAP_STATE_DRAGGING);

    /* At t=6640ms (140ms inactive == 140ms grace), drag releases cleanly */
    act = alps_tap_gesture_handle_timeout(&ctx, &cfg.tap, 6640, 0);
    assert(act.type == ALPS_ACTION_BUTTON_UP);
    assert(ctx.state == ALPS_TAP_STATE_IDLE);

    printf("  [PASS] test_tap_and_drag_across_zero_pressure_shear\n");
}

static void test_saturating_math_safety(void) {
    assert(alps_add_motion_saturated(0, INT32_MIN, 10000) == 10000);
    assert(alps_add_motion_saturated(9995, -20, 10000) == 10000);
    assert(alps_add_motion_saturated(10, -7, 10000) == 17);
    printf("  [PASS] test_saturating_math_safety\n");
}

int main(void) {
    printf("Running ALPS Trackstick Transducer Unit Tests (v%s)...\n", ALPS_DAEMON_VERSION);
    test_single_tap_and_anti_drift();
    test_micro_nudge_force_ratio_rejection();
    test_high_velocity_navigation_rejection();
    test_double_click_zero_latency_and_release();
    test_ultra_fast_rebound_during_tap1_hold();
    test_click_then_steer_cancellation();
    test_tap_and_drag_across_zero_pressure_shear();
    test_saturating_math_safety();
    printf("All 8 test suites passed!\n");
    return 0;
}
