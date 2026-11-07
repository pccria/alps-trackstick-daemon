/**
 * @file test_calibrate.c
 * @brief Deterministic unit tests for the interactive calibration feature.
 *
 * Validates:
 * 1. Tracker initialization and reset states.
 * 2. Event feeding, impulse tap duration calculation, and peak force tracking.
 * 3. ASCII visualizer bar formatting across dynamic ranges.
 * 4. Algorithmic threshold recommendation math based on observed physical tap data.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "core/config.h"
#include "core/event.h"
#include "features/calibrate.h"

static alps_event_t make_ev(alps_event_type_t type, int32_t value, uint64_t ts_ms) {
    alps_event_t ev;
    ev.type = type;
    ev.value = value;
    ev.timestamp_ms = ts_ms;
    return ev;
}

static void test_calibrate_tracker_init(void) {
    alps_calibrate_tracker_t tracker;
    alps_calibrate_tracker_init(&tracker);

    assert(tracker.current_z == 0);
    assert(tracker.current_dx == 0);
    assert(tracker.current_dy == 0);
    assert(tracker.max_tap_z == 0);
    assert(tracker.min_tap_z == 255);
    assert(tracker.tap_count == 0);
    assert(tracker.tap_peak_sum == 0);
    assert(tracker.tap_duration_sum == 0);

    printf("  [PASS] test_calibrate_tracker_init\n");
}

static void test_calibrate_tracker_feed_tap(void) {
    alps_calibrate_tracker_t tracker;
    alps_calibrate_tracker_init(&tracker);

    /* Tap 1: contact at t=1000ms with Z=20, peak Z=36 at 1030ms, release Z=0 at 1060ms (60ms duration) */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 20, 1000);
    int completed = alps_calibrate_tracker_feed_event(&tracker, &ev, 1000);
    assert(completed == 0);
    assert(tracker.active_peak_z == 20);

    ev = make_ev(ALPS_EV_PRESSURE, 36, 1030);
    completed = alps_calibrate_tracker_feed_event(&tracker, &ev, 1030);
    assert(completed == 0);
    assert(tracker.active_peak_z == 36);

    ev = make_ev(ALPS_EV_PRESSURE, 0, 1060);
    completed = alps_calibrate_tracker_feed_event(&tracker, &ev, 1060);
    assert(completed == 1);
    assert(tracker.tap_count == 1);
    assert(tracker.max_tap_z == 36);
    assert(tracker.min_tap_z == 36);
    assert(tracker.tap_peak_sum == 36);
    assert(tracker.tap_duration_sum == 60);

    /* Tap 2: lighter tap with peak Z=28 and duration 45ms */
    ev = make_ev(ALPS_EV_PRESSURE, 28, 1200);
    alps_calibrate_tracker_feed_event(&tracker, &ev, 1200);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 1245);
    completed = alps_calibrate_tracker_feed_event(&tracker, &ev, 1245);
    assert(completed == 1);
    assert(tracker.tap_count == 2);
    assert(tracker.max_tap_z == 36);
    assert(tracker.min_tap_z == 28);
    assert(tracker.tap_peak_sum == 64);
    assert(tracker.tap_duration_sum == 105);

    printf("  [PASS] test_calibrate_tracker_feed_tap\n");
}

static void test_calibrate_format_bar(void) {
    char buf[64];

    /* 0% deflection */
    alps_calibrate_format_bar(0, 100, 20, buf, sizeof(buf));
    assert(strcmp(buf, "[                    ]") == 0);

    /* 50% deflection */
    alps_calibrate_format_bar(50, 100, 20, buf, sizeof(buf));
    assert(strcmp(buf, "[=========>          ]") == 0);

    /* 100% deflection */
    alps_calibrate_format_bar(100, 100, 20, buf, sizeof(buf));
    assert(strcmp(buf, "[===================>]") == 0);

    printf("  [PASS] test_calibrate_format_bar\n");
}

static void test_calibrate_recommendation_math(void) {
    alps_calibrate_tracker_t tracker;
    alps_calibrate_tracker_init(&tracker);

    int tap1 = 0, tap2 = 0, release = 0;

    /* Without taps, defaults should be produced */
    alps_calibrate_compute_recommendation(&tracker, &tap1, &tap2, &release);
    assert(tap1 == ALPS_DEFAULT_TAP1_PRESSURE);
    assert(tap2 == ALPS_DEFAULT_TAP2_REBOUND_PRESSURE);
    assert(release == ALPS_DEFAULT_RELEASE_PRESSURE);

    /* Record 3 taps with peaks 32, 36, 40 (average peak = 36) */
    alps_event_t ev = make_ev(ALPS_EV_PRESSURE, 32, 1000);
    alps_calibrate_tracker_feed_event(&tracker, &ev, 1000);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 1050);
    alps_calibrate_tracker_feed_event(&tracker, &ev, 1050);

    ev = make_ev(ALPS_EV_PRESSURE, 36, 1200);
    alps_calibrate_tracker_feed_event(&tracker, &ev, 1200);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 1250);
    alps_calibrate_tracker_feed_event(&tracker, &ev, 1250);

    ev = make_ev(ALPS_EV_PRESSURE, 40, 1400);
    alps_calibrate_tracker_feed_event(&tracker, &ev, 1400);
    ev = make_ev(ALPS_EV_PRESSURE, 0, 1450);
    alps_calibrate_tracker_feed_event(&tracker, &ev, 1450);

    assert(tracker.tap_count == 3);

    /* Expected: avg_peak = 36
     * tap1 = (36 * 70) / 100 = 25
     * tap2 = (25 * 68) / 100 = 17
     * release = (25 * 45) / 100 = 11 */
    alps_calibrate_compute_recommendation(&tracker, &tap1, &tap2, &release);
    assert(tap1 == 25);
    assert(tap2 == 17);
    assert(release == 11);

    printf("  [PASS] test_calibrate_recommendation_math\n");
}

int main(void) {
    printf("Running ALPS Trackstick Calibration Unit Tests (v%s)...\n", ALPS_DAEMON_VERSION);
    test_calibrate_tracker_init();
    test_calibrate_tracker_feed_tap();
    test_calibrate_format_bar();
    test_calibrate_recommendation_math();
    printf("All 4 calibration test suites passed!\n");
    return 0;
}
