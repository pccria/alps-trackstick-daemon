/**
 * @file test_scroll_wheel.c
 * @brief Deterministic unit tests for the 2D kinetic trackstick scroll wheel module.
 *
 * Validates:
 * 1. Middle-button engagement & release lifecycle.
 * 2. Continuous high-resolution vertical scrolling & counter-drift cursor lock.
 * 3. Continuous high-resolution horizontal scrolling & counter-drift cursor lock.
 * 4. Primary axis-lock deadband filtering (suppressing lateral tremor during vertical reading).
 * 5. Axis-lock breakout when cross-axis deflection exceeds deadband.
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

static void test_vertical_scrolling_and_counter_drift(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 2000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Deflect stick UP (REL_Y = -3) -> scroll UP */
    ev = make_ev(ALPS_EV_REL_Y, -3, 2010);
    alps_action_t act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    assert(act.type == ALPS_ACTION_SCROLL);
    assert(act.wheel_v_hires == 3 * cfg.scroll.hires_multiplier); /* +45 */
    assert(act.wheel_h_hires == 0);
    assert(act.drift_y == 3); /* Counter-drift opposite to -3 */
    assert(act.drift_x == 0);

    /* Deflect stick DOWN (REL_Y = +2) -> scroll DOWN */
    ev = make_ev(ALPS_EV_REL_Y, 2, 2020);
    act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    assert(act.type == ALPS_ACTION_SCROLL);
    assert(act.wheel_v_hires == -2 * cfg.scroll.hires_multiplier); /* -30 */
    assert(act.drift_y == -2);

    printf("  [PASS] test_vertical_scrolling_and_counter_drift\n");
}

static void test_horizontal_scrolling_and_counter_drift(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 3000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Deflect stick RIGHT (REL_X = +4) -> scroll RIGHT */
    ev = make_ev(ALPS_EV_REL_X, 4, 3010);
    alps_action_t act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    assert(act.type == ALPS_ACTION_SCROLL);
    assert(act.wheel_h_hires == 4 * cfg.scroll.hires_multiplier); /* +60 */
    assert(act.wheel_v_hires == 0);
    assert(act.drift_x == -4);
    assert(act.drift_y == 0);

    printf("  [PASS] test_horizontal_scrolling_and_counter_drift\n");
}

static void test_axis_lock_deadband_filtering(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 4000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* Vertical deflection locks axis to VERTICAL */
    ev = make_ev(ALPS_EV_REL_Y, -5, 4010);
    alps_action_t act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.wheel_v_hires == 75);
    assert(ctx.axis_lock == ALPS_AXIS_LOCK_VERTICAL);

    /* Incidental lateral tremor (REL_X = 1) under deadband threshold (4) */
    ev = make_ev(ALPS_EV_REL_X, 1, 4015);
    act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.type == ALPS_ACTION_SCROLL);
    assert(act.wheel_h_hires == 0); /* Suppressed! */
    assert(act.drift_x == -1);      /* Counter-drift still anchors pointer */

    /* Second tremor (REL_X = 2, total cross-axis = 3 < 4) */
    ev = make_ev(ALPS_EV_REL_X, 2, 4020);
    act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.wheel_h_hires == 0); /* Still suppressed */
    assert(ctx.axis_lock == ALPS_AXIS_LOCK_VERTICAL);

    /* Intentional lateral steering (REL_X = 2, total cross-axis = 5 >= 4) */
    ev = make_ev(ALPS_EV_REL_X, 2, 4025);
    act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.wheel_h_hires == 2 * cfg.scroll.hires_multiplier); /* Now unlocked! */
    assert(ctx.axis_lock == ALPS_AXIS_LOCK_NONE);

    printf("  [PASS] test_axis_lock_deadband_filtering\n");
}

static void test_legacy_detent_accumulation(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t ctx;
    alps_scroll_wheel_init(&ctx);

    /* Engage scroll */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 5000);
    alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);

    /* First step: -4 Y -> 60 hi-res units (< 120 -> wheel_v == 0) */
    ev = make_ev(ALPS_EV_REL_Y, -4, 5010);
    alps_action_t act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.wheel_v_hires == 60);
    assert(act.wheel_v == 0);

    /* Second step: -4 Y -> another 60 hi-res units (total 120 -> wheel_v == 1!) */
    ev = make_ev(ALPS_EV_REL_Y, -4, 5020);
    act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
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

    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 6000);
    alps_action_t act = alps_scroll_wheel_process_event(&ctx, &cfg.scroll, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(alps_scroll_wheel_is_engaged(&ctx) == 0);

    printf("  [PASS] test_scroll_disabled_configuration\n");
}

int main(void) {
    printf("Running ALPS Trackstick 2D Kinetic Scroll Unit Tests (v%s)...\n", ALPS_DAEMON_VERSION);
    test_scroll_wheel_engage_and_release();
    test_vertical_scrolling_and_counter_drift();
    test_horizontal_scrolling_and_counter_drift();
    test_axis_lock_deadband_filtering();
    test_legacy_detent_accumulation();
    test_scroll_disabled_configuration();
    printf("All 6 scroll wheel test suites passed!\n");
    return 0;
}
