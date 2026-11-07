/**
 * @file test_scroll_wheel.c
 * @brief Deterministic unit tests for the 2D kinetic trackstick scroll wheel module.
 *
 * Validates:
 * 1. Middle-button engagement & release lifecycle.
 * 2. Continuous high-resolution vertical scrolling & lateral tremor rejection.
 * 3. Continuous high-resolution horizontal scrolling & vertical tremor rejection.
 * 4. Dual-range progressive ballistics (micro-stepping precision vs high-speed glide).
 * 5. Linear scroll fallback when progressive mode is toggled off.
 * 6. Transient mid-drag direction transitions (vertical -> horizontal -> vertical).
 * 7. Simultaneous 2D diagonal panning when steering diagonally.
 * 8. Continuous C^1 Hermite angle blending during diagonal transitions.
 * 9. Discrete legacy detent notch accumulation (120 hi-res units = 1 notch).
 * 10. Feature toggle disabling scroll when configured off.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "core/config.h"
#include "core/event.h"
#include "core/action.h"
#include "features/scroll_wheel.h"

static alps_event_t make_ev(alps_event_type_t type, int32_t value, uint64_t ts_ms) {
    alps_event_t ev;
    ev.type = type;
    ev.value = value;
    ev.timestamp_ms = ts_ms;
    return ev;
}

static void test_scroll_wheel_engage_and_release(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);
    assert(alps_scroll_wheel_is_engaged(&ctx) == 0);

    /* Middle button down */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 1000);
    alps_action_t act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(alps_scroll_wheel_is_engaged(&ctx) == 1);

    /* Middle button up */
    ev = make_ev(ALPS_EV_BTN_MIDDLE, 0, 1100);
    act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(alps_scroll_wheel_is_engaged(&ctx) == 0);

    printf("  [PASS] test_scroll_wheel_engage_and_release\n");
}

static void test_vertical_scrolling_with_lateral_tremor_suppression(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 2000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Deflect stick DOWN (+6 Y) with natural slight lateral tremor (+1 X) */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 1, 6, 0);

    assert(act.type == ALPS_ACTION_SCROLL);
    /* For mag=6: 6*12 + (16*6/4) = 72 + 24 = 96 */
    assert(act.wheel_v_hires == -96); /* Scroll down */
    assert(act.wheel_h_hires == 0);   /* Lateral tremor suppressed! */

    printf("  [PASS] test_vertical_scrolling_with_lateral_tremor_suppression\n");
}

static void test_horizontal_scrolling_with_vertical_tremor_suppression(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 3000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Deflect stick RIGHT (+8 X) with slight vertical wobble (-2 Y) */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 8, -2, 0);

    assert(act.type == ALPS_ACTION_SCROLL);
    /* For mag=8: 8*12 + (36*6/4) = 96 + 54 = 150 */
    assert(act.wheel_h_hires == 150); /* Scroll right */
    assert(act.wheel_v_hires == 0);   /* Vertical wobble suppressed! */

    printf("  [PASS] test_horizontal_scrolling_with_vertical_tremor_suppression\n");
}

static void test_progressive_ballistics_curve(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 4000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Sub-notch micro-stepping (mag=1 -> 12 units = exactly 10% of a notch) */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -1, 0);
    assert(act.wheel_v_hires == 12);

    /* mag=2 -> 24 units = exactly 20% of a notch */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -2, 0);
    assert(act.wheel_v_hires == 24);

    /* High deflection (mag=8 -> 150 units = >1.2 full notches per frame) */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -8, 0);
    assert(act.wheel_v_hires == 150);

    printf("  [PASS] test_progressive_ballistics_curve\n");
}

static void test_linear_scroll_fallback(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);
    cfg.scroll.progressive_enabled = 0; /* Force linear scaling */

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 4500);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Linear: mag=8 -> 8 * 12 = 96 units */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -8, 0);
    assert(act.wheel_v_hires == 96);

    printf("  [PASS] test_linear_scroll_fallback\n");
}

static void test_transient_mid_drag_direction_transition(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 5000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Frame 1: Pure vertical scroll UP (-5 Y) -> mag=5: 5*12 + (9*6/4) = 73 */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -5, 0);
    assert(act.wheel_v_hires == 73);
    assert(act.wheel_h_hires == 0);

    /* Frame 2: Seamlessly transition mid-drag to pure horizontal RIGHT (+6 X) -> mag=6: 96 */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 6, 0, 0);
    assert(act.wheel_v_hires == 0);
    assert(act.wheel_h_hires == 96);

    /* Frame 3: Seamlessly transition mid-drag back to vertical DOWN (+5 Y) -> -73 */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, 5, 0);
    assert(act.wheel_v_hires == -73);
    assert(act.wheel_h_hires == 0);

    printf("  [PASS] test_transient_mid_drag_direction_transition\n");
}

static void test_true_diagonal_simultaneous_panning(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 6000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Steer diagonally: +4 X, -5 Y (neither axis dominates by 2x) */
    /* mag=4: 4*12 + (4*6/4) = 54 */
    /* mag=5: 5*12 + (9*6/4) = 73 */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 4, -5, 0);

    assert(act.type == ALPS_ACTION_SCROLL);
    assert(act.wheel_h_hires == 54);
    assert(act.wheel_v_hires == 73);

    printf("  [PASS] test_true_diagonal_simultaneous_panning\n");
}

static void test_continuous_hermite_angle_blending(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 6500);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Test 1: dx = 3, dy = -6 (ratio 1:2 -> exactly t = 0.5 in Hermite ramp)
     * Vertical dominant: dy = -6 gives hires = 96
     * Lateral cross: dx = 3 at t = 0.5 has S(0.5) = 0.5 -> eff_x = (3*128 + 128)/256 = 2
     * Ballistics for eff_x = 2: 2 * 12 = 24 */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 3, -6, 0);
    assert(act.type == ALPS_ACTION_SCROLL);
    assert(act.wheel_v_hires == 96);
    assert(act.wheel_h_hires == 24);

    /* Test 2: dx = -6, dy = 3 (horizontal dominant, ratio 1:2 -> exactly t = 0.5)
     * eff_y = -2, wheel_v_hires = -24, wheel_h_hires = -96 */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, -6, 3, 0);
    assert(act.type == ALPS_ACTION_SCROLL);
    assert(act.wheel_h_hires == -96);
    assert(act.wheel_v_hires == -24);

    printf("  [PASS] test_continuous_hermite_angle_blending\n");
}

static void test_legacy_detent_accumulation(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 7000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Step 1: -4 Y -> 54 hi-res units (< 120 -> wheel_v == 0) */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -4, 0);
    assert(act.wheel_v_hires == 54);
    assert(act.wheel_v == 0);

    /* Step 2: -6 Y -> 96 hi-res units (total 150 -> wheel_v == 1, remainder 30!) */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -6, 0);
    assert(act.wheel_v_hires == 96);
    assert(act.wheel_v == 1);
    assert(ctx.acc_v_hires == 30);

    printf("  [PASS] test_legacy_detent_accumulation\n");
}

static void test_scroll_disabled_configuration(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);
    cfg.scroll.enabled = 0;

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 8000);
    alps_action_t act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(alps_scroll_wheel_is_engaged(&ctx) == 0);

    printf("  [PASS] test_scroll_disabled_configuration\n");
}

int main(void) {
    printf("Running ALPS Trackstick 2D Kinetic Scroll Unit Tests (v%s)...\n", ALPS_DAEMON_VERSION);
    test_scroll_wheel_engage_and_release();
    test_vertical_scrolling_with_lateral_tremor_suppression();
    test_horizontal_scrolling_with_vertical_tremor_suppression();
    test_progressive_ballistics_curve();
    test_linear_scroll_fallback();
    test_transient_mid_drag_direction_transition();
    test_true_diagonal_simultaneous_panning();
    test_continuous_hermite_angle_blending();
    test_legacy_detent_accumulation();
    test_scroll_disabled_configuration();
    printf("All 10 scroll wheel test suites passed!\n");
    return 0;
}
