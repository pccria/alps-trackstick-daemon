/**
 * @file test_calibrate.c
 * @brief Deterministic unit tests for the interactive calibration feature.
 *
 * Validates:
 * 1. Tracker initialization and reset states.
 * 2. Event feeding, impulse tap duration calculation, and peak force tracking.
 * 3. ASCII visualizer bar formatting across dynamic ranges.
 * 4. Algorithmic threshold recommendation math for legacy and guided modes.
 * 5. Guided profile container initialization and derived metrics.
 * 6. High-noise floor gating and rebound clamp safety.
 * 7. INI configuration file generation from fresh calibration profile.
 * 8. Non-destructive INI merging into existing configurations.
 * 9. Permission denied and invalid path safety.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>

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

static void test_calibrate_guided_profile_init(void) {
    alps_guided_profile_t prof;
    alps_calibrate_profile_init(&prof);

    assert(prof.noise_floor_z == 0);
    assert(prof.noise_floor_max_z == 0);
    assert(prof.single_tap_count == 0);
    assert(prof.single_tap_min_peak == 255);
    assert(prof.rec_tap1_pressure == ALPS_DEFAULT_TAP1_PRESSURE);
    assert(prof.rec_tap2_pressure == ALPS_DEFAULT_TAP2_REBOUND_PRESSURE);
    assert(prof.rec_release_pressure == ALPS_DEFAULT_RELEASE_PRESSURE);
    assert(prof.rec_tap_duration_ms == ALPS_DEFAULT_MAX_TAP_DURATION_MS);
    assert(prof.rec_double_window_ms == ALPS_DEFAULT_DOUBLE_TAP_WINDOW_MS);
    assert(prof.rec_tap_motion == ALPS_DEFAULT_MAX_TAP_MOTION);

    printf("  [PASS] test_calibrate_guided_profile_init\n");
}

static void test_calibrate_guided_recommendations_math(void) {
    alps_guided_profile_t prof;
    alps_calibrate_profile_init(&prof);

    /* Case 1: Empty profile returns defaults */
    alps_calibrate_compute_guided_recommendations(&prof);
    assert(prof.rec_tap1_pressure == ALPS_DEFAULT_TAP1_PRESSURE);
    assert(prof.rec_tap2_pressure == ALPS_DEFAULT_TAP2_REBOUND_PRESSURE);
    assert(prof.rec_release_pressure == ALPS_DEFAULT_RELEASE_PRESSURE);

    /* Case 2: Realistic profile with resting noise floor = 2, max spike = 4 */
    prof.noise_floor_z = 2;
    prof.noise_floor_max_z = 4;
    prof.single_tap_count = 5;
    prof.single_tap_avg_peak = 40;
    prof.single_tap_min_peak = 35;
    prof.single_tap_max_peak = 46;
    prof.single_tap_avg_dur_ms = 55;

    prof.double_tap_count = 3;
    prof.double_tap1_avg_peak = 42;
    prof.double_tap2_avg_peak = 30;
    prof.double_tap_avg_gap_ms = 110;

    prof.steer_max_z = 50;
    prof.steer_max_motion = 24;

    alps_calibrate_compute_guided_recommendations(&prof);

    /* Calculations:
     * t1 = (40 * 70) / 100 = 28. (Noise floor max 4 + 8 = 12 <= 28 <= min_peak 35) -> 28
     * t2 = (30 * 70) / 100 = 21. (Noise floor max 4 + 5 = 9 <= 21 < t1 28) -> 21
     * rel = (28 * 45) / 100 = 12. (Noise floor max 4 + 2 = 6 <= 12 < t2 21) -> 12
     * dur = 55 + (55 * 40 / 100) = 77 -> clamped to min 120 ms
     * win = 110 + (110 * 50 / 100) = 165 -> clamped to min 180 ms
     * mot = ALPS_DEFAULT_MAX_TAP_MOTION (8)
     */
    assert(prof.rec_tap1_pressure == 28);
    assert(prof.rec_tap2_pressure == 21);
    assert(prof.rec_release_pressure == 12);
    assert(prof.rec_tap_duration_ms == 120);
    assert(prof.rec_double_window_ms == 180);
    assert(prof.rec_tap_motion == 8);

    /* Case 3: High noise floor clamping */
    prof.noise_floor_z = 10;
    prof.noise_floor_max_z = 18;
    prof.single_tap_avg_peak = 25; /* 25 * 0.7 = 17, but noise_max + 8 = 26 */
    prof.single_tap_min_peak = 24;
    alps_calibrate_compute_guided_recommendations(&prof);

    /* t1 must be at least noise_max + 8 = 26 */
    assert(prof.rec_tap1_pressure >= 26);
    /* release must be at least noise_max + 2 = 20 */
    assert(prof.rec_release_pressure >= 20);

    printf("  [PASS] test_calibrate_guided_recommendations_math\n");
}

static void test_calibrate_save_config_fresh_file(void) {
    const char *tmp_path = "/tmp/test_alps_guided_fresh.conf";
    unlink(tmp_path);

    alps_guided_profile_t prof;
    alps_calibrate_profile_init(&prof);
    prof.rec_tap1_pressure = 29;
    prof.rec_tap2_pressure = 19;
    prof.rec_release_pressure = 13;
    prof.rec_tap_duration_ms = 160;
    prof.rec_tap_motion = 7;
    prof.rec_double_window_ms = 220;

    int ret = alps_calibrate_save_config(tmp_path, &prof);
    assert(ret == 0);

    /* Parse saved file with core configuration loader */
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);
    int load_ret = alps_config_load_file(&cfg, tmp_path, 1);
    assert(load_ret == 0);

    assert(cfg.tap.tap1_pressure == 29);
    assert(cfg.tap.tap2_rebound_pressure == 19);
    assert(cfg.tap.release_pressure == 13);
    assert(cfg.tap.max_tap_duration_ms == 160);
    assert(cfg.tap.max_tap_motion == 7);
    assert(cfg.tap.double_tap_window_ms == 220);
    assert(cfg.scroll.enabled == 1);

    unlink(tmp_path);
    printf("  [PASS] test_calibrate_save_config_fresh_file\n");
}

static void test_calibrate_save_config_merge_existing(void) {
    const char *tmp_path = "/tmp/test_alps_guided_merge.conf";
    unlink(tmp_path);

    /* Create pre-existing configuration file with custom settings and comments */
    FILE *f = fopen(tmp_path, "w");
    assert(f != NULL);
    fprintf(f, "# User custom configuration\n");
    fprintf(f, "[general]\n");
    fprintf(f, "verbose = true\n\n");
    fprintf(f, "[tap]\n");
    fprintf(f, "tap1_pressure = 15\n");
    fprintf(f, "drag_start_motion = 88\n");
    fprintf(f, "# Pre-existing comment inside tap\n");
    fprintf(f, "click_hold_ms = 35\n\n");
    fprintf(f, "[scroll]\n");
    fprintf(f, "enabled = false\n");
    fprintf(f, "hires_multiplier = 9\n");
    fclose(f);

    alps_guided_profile_t prof;
    alps_calibrate_profile_init(&prof);
    prof.rec_tap1_pressure = 33;
    prof.rec_tap2_pressure = 23;
    prof.rec_release_pressure = 15;
    prof.rec_tap_duration_ms = 195;
    prof.rec_tap_motion = 9;
    prof.rec_double_window_ms = 290;

    int ret = alps_calibrate_save_config(tmp_path, &prof);
    assert(ret == 0);

    /* Validate merged configuration */
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);
    int load_ret = alps_config_load_file(&cfg, tmp_path, 1);
    assert(load_ret == 0);

    /* Calibrated values overwritten */
    assert(cfg.tap.tap1_pressure == 33);
    assert(cfg.tap.tap2_rebound_pressure == 23);
    assert(cfg.tap.release_pressure == 15);
    assert(cfg.tap.max_tap_duration_ms == 195);
    assert(cfg.tap.max_tap_motion == 9);
    assert(cfg.tap.double_tap_window_ms == 290);

    /* Existing non-calibrated keys preserved */
    assert(cfg.verbose == 1);
    assert(cfg.tap.drag_start_motion == 88);
    assert(cfg.tap.click_hold_ms == 35);
    assert(cfg.scroll.enabled == 0);
    assert(cfg.scroll.hires_multiplier == 9);

    /* Read raw file to verify comment preservation */
    FILE *rf = fopen(tmp_path, "r");
    assert(rf != NULL);
    char buf[2048];
    size_t n = fread(buf, 1, sizeof(buf) - 1, rf);
    buf[n] = '\0';
    fclose(rf);

    assert(strstr(buf, "# User custom configuration") != NULL);
    assert(strstr(buf, "# Pre-existing comment inside tap") != NULL);

    unlink(tmp_path);
    printf("  [PASS] test_calibrate_save_config_merge_existing\n");
}

static void test_calibrate_save_config_permission_denied(void) {
    alps_guided_profile_t prof;
    alps_calibrate_profile_init(&prof);

    /* Attempt to write to a guaranteed read-only/uncreatable path */
    int ret = alps_calibrate_save_config("/proc/sys/alps_non_existent.conf", &prof);
    assert(ret == -2 || ret == -1);

    printf("  [PASS] test_calibrate_save_config_permission_denied\n");
}

int main(void) {
    printf("Running ALPS Trackstick Calibration Unit Tests (v%s)...\n", ALPS_DAEMON_VERSION);
    test_calibrate_tracker_init();
    test_calibrate_tracker_feed_tap();
    test_calibrate_format_bar();
    test_calibrate_recommendation_math();
    test_calibrate_guided_profile_init();
    test_calibrate_guided_recommendations_math();
    test_calibrate_save_config_fresh_file();
    test_calibrate_save_config_merge_existing();
    test_calibrate_save_config_permission_denied();
    printf("All 9 calibration test suites passed!\n");
    return 0;
}
