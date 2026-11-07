/**
 * @file test_scroll_wheel.c
 * @brief Deterministic unit tests for the 2D kinetic trackstick scroll wheel module.
 *
 * Validates:
 * 1. Middle-button engagement & release lifecycle.
 * 2. Continuous high-resolution vertical scrolling & lateral tremor rejection.
 * 3. Continuous high-resolution horizontal scrolling & vertical tremor rejection.
 * 4. Transient mid-drag direction transitions (vertical -> horizontal -> vertical).
 * 5. Simultaneous 2D diagonal panning when steering diagonally.
 * 6. Discrete legacy detent notch accumulation (120 hi-res units = 1 notch).
 * 7. Feature toggle disabling scroll when configured off.
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
    assert(act.wheel_v_hires == -6 * cfg.scroll.hires_multiplier); /* -90 (scroll down) */
    assert(act.wheel_h_hires == 0);                                /* Lateral tremor suppressed! */
    assert(act.drift_x == -1);                                     /* Pointer still locked */
    assert(act.drift_y == -6);

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
    assert(act.wheel_h_hires == 8 * cfg.scroll.hires_multiplier); /* +120 (scroll right) */
    assert(act.wheel_v_hires == 0);                               /* Vertical wobble suppressed! */
    assert(act.drift_x == -8);
    assert(act.drift_y == 2);

    printf("  [PASS] test_horizontal_scrolling_with_vertical_tremor_suppression\n");
}

static void test_transient_mid_drag_direction_transition(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 4000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Frame 1: Pure vertical scroll UP (-5 Y) */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -5, 0);
    assert(act.wheel_v_hires == 75);
    assert(act.wheel_h_hires == 0);

    /* Frame 2: Seamlessly transition mid-drag to pure horizontal RIGHT (+6 X) */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 6, 0, 0);
    assert(act.wheel_v_hires == 0);
    assert(act.wheel_h_hires == 90);

    /* Frame 3: Seamlessly transition mid-drag back to vertical DOWN (+5 Y) */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, 5, 0);
    assert(act.wheel_v_hires == -75);
    assert(act.wheel_h_hires == 0);

    printf("  [PASS] test_transient_mid_drag_direction_transition\n");
}

static void test_true_diagonal_simultaneous_panning(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 5000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Steer diagonally: +4 X, -5 Y (neither axis dominates by 2x) */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 4, -5, 0);

    assert(act.type == ALPS_ACTION_SCROLL);
    assert(act.wheel_h_hires == 4 * cfg.scroll.hires_multiplier); /* +60 */
    assert(act.wheel_v_hires == 5 * cfg.scroll.hires_multiplier); /* +75 */
    assert(act.drift_x == -4);
    assert(act.drift_y == 5);

    printf("  [PASS] test_true_diagonal_simultaneous_panning\n");
}

static void test_legacy_detent_accumulation(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 6000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Step 1: -4 Y -> 60 hi-res units (< 120 -> wheel_v == 0) */
    alps_action_t act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -4, 0);
    assert(act.wheel_v_hires == 60);
    assert(act.wheel_v == 0);

    /* Step 2: -4 Y -> another 60 hi-res units (total 120 -> wheel_v == 1!) */
    act = alps_scroll_wheel_process_frame(&ctx, &cfg.scroll, 0, -4, 0);
    assert(act.wheel_v_hires == 60);
    assert(act.wheel_v == 1);

    printf("  [PASS] test_legacy_detent_accumulation\n");
}

static void test_scroll_disabled_configuration(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);
    cfg.scroll.enabled = 0;

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 7000);
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
    test_transient_mid_drag_direction_transition();
    test_true_diagonal_simultaneous_panning();
    test_legacy_detent_accumulation();
    test_scroll_disabled_configuration();
    printf("All 7 scroll wheel test suites passed!\n");
    return 0;
}
