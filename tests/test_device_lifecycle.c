/**
 * @file test_device_lifecycle.c
 * @brief Deterministic unit tests for device monitoring and suspend/resume lifecycle safety.
 *
 * Validates:
 * 1. Inotify device monitor creation, non-blocking drain, and teardown.
 * 2. Virtual button-release safety on simulated disconnect during Tap 1 Hold.
 * 3. Virtual button-release safety on simulated disconnect during Active Drag.
 * 4. Scroll wheel engagement and accumulator reset on simulated disconnect.
 * 5. Edge cases: invalid file descriptor handling and error resilience.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>

#include "core/config.h"
#include "core/event.h"
#include "core/action.h"
#include "features/tap_gesture.h"
#include "features/scroll_wheel.h"
#include "io/device.h"

static alps_event_t make_ev(alps_event_type_t type, int32_t value, uint64_t ts_ms) {
    alps_event_t ev;
    ev.type = type;
    ev.value = value;
    ev.timestamp_ms = ts_ms;
    return ev;
}

static void test_device_monitor_lifecycle(void) {
    int ifd = alps_device_monitor_create();
    assert(ifd >= 0);

    /* Verify non-blocking behavior: draining when empty returns 0 immediately */
    int drained = alps_device_monitor_drain(ifd);
    assert(drained == 0);

    alps_device_monitor_destroy(ifd);
    printf("  [PASS] test_device_monitor_lifecycle\n");
}

static void test_device_disconnect_tap_hold_release(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t tap_ctx;
    alps_tap_gesture_init(&tap_ctx);

    /* Initiate contact: Z=30 */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 30, 1000);
    alps_tap_gesture_process_event(&tap_ctx, &cfg.tap, &ev, 0);

    /* Release contact: Z=0 within 180ms -> enters TAP1_HOLD with button down */
    ev = make_ev(ALPS_EV_PRESSURE, 0, 1050);
    alps_action_t act = alps_tap_gesture_process_event(&tap_ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_BUTTON_DOWN);
    assert(tap_ctx.state == ALPS_TAP_STATE_TAP1_HOLD);

    /* Simulate device disconnect / suspend in the middle of hold */
    alps_action_t rst_act = alps_tap_gesture_reset(&tap_ctx);
    assert(rst_act.type == ALPS_ACTION_BUTTON_UP);
    assert(tap_ctx.state == ALPS_TAP_STATE_IDLE);

    printf("  [PASS] test_device_disconnect_tap_hold_release\n");
}

static void test_device_disconnect_active_drag_release(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_tap_gesture_t tap_ctx;
    alps_tap_gesture_init(&tap_ctx);

    /* Tap 1 */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 30, 2000);
    alps_tap_gesture_process_event(&tap_ctx, &cfg.tap, &ev, 0);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 2050);
    alps_tap_gesture_process_event(&tap_ctx, &cfg.tap, &ev, 0);

    /* Advance hold timeout to enter TAP1_POST_HOLD (primed) */
    alps_tap_gesture_handle_timeout(&tap_ctx, &cfg.tap, 2100, 0);

    /* Tap 2 contact -> Button down emitted */
    ev = make_ev(ALPS_EV_PRESSURE, 25, 2150);
    alps_action_t act = alps_tap_gesture_process_event(&tap_ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_BUTTON_DOWN);
    assert(tap_ctx.state == ALPS_TAP_STATE_TAP2_DOWN);

    /* Motion shear -> locks DRAGGING */
    ev = make_ev(ALPS_EV_REL_X, 10, 2160);
    act = alps_tap_gesture_process_event(&tap_ctx, &cfg.tap, &ev, 0);
    assert(act.type == ALPS_ACTION_NONE);
    assert(tap_ctx.state == ALPS_TAP_STATE_DRAGGING);

    /* Disconnect while user was dragging a window */
    alps_action_t rst_act = alps_tap_gesture_reset(&tap_ctx);
    assert(rst_act.type == ALPS_ACTION_BUTTON_UP);
    assert(tap_ctx.state == ALPS_TAP_STATE_IDLE);

    printf("  [PASS] test_device_disconnect_active_drag_release\n");
}

static void test_device_disconnect_scroll_reset(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    alps_scroll_wheel_t scroll_ctx;
    alps_scroll_wheel_init(&scroll_ctx);

    /* Engage middle button */
    alps_event_t ev = make_ev(ALPS_EV_BTN_MIDDLE, 1, 3000);
    alps_scroll_wheel_process_event(&scroll_ctx, &cfg.scroll, &ev, 0);
    assert(alps_scroll_wheel_is_engaged(&scroll_ctx) == 1);

    /* Accumulate partial sub-notch */
    alps_scroll_wheel_process_frame(&scroll_ctx, &cfg.scroll, 0, -2, 0);
    assert(scroll_ctx.acc_v_hires != 0);

    /* Reset on disconnect */
    alps_scroll_wheel_reset(&scroll_ctx);
    assert(alps_scroll_wheel_is_engaged(&scroll_ctx) == 0);
    assert(scroll_ctx.acc_v_hires == 0);
    assert(scroll_ctx.acc_h_hires == 0);

    printf("  [PASS] test_device_disconnect_scroll_reset\n");
}

static void test_device_monitor_invalid_fd_safety(void) {
    /* Safe error handling when passed invalid file descriptors */
    assert(alps_device_monitor_drain(-1) == -1);
    alps_device_monitor_destroy(-1);

    printf("  [PASS] test_device_monitor_invalid_fd_safety\n");
}

int main(void) {
    printf("Running ALPS Trackstick Device Lifecycle Unit Tests (v%s)...\n", ALPS_DAEMON_VERSION);
    test_device_monitor_lifecycle();
    test_device_disconnect_tap_hold_release();
    test_device_disconnect_active_drag_release();
    test_device_disconnect_scroll_reset();
    test_device_monitor_invalid_fd_safety();
    printf("All 5 device lifecycle test suites passed!\n");
    return 0;
}
