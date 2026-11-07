/**
 * @file test_config_file.c
 * @brief Deterministic unit tests for INI configuration file parsing and precedence.
 *
 * Validates:
 * 1. Default config non-fatal absence vs explicit `-c` path error handling.
 * 2. Section header and flat key-value parsing ([general], [tap], [scroll]).
 * 3. Boolean variant flexibility (true/false, yes/no, on/off, 1/0).
 * 4. Inline comment stripping and whitespace trimming.
 * 5. Precedence: Defaults < Config File < CLI Flags.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>

#include "core/config.h"

static void test_missing_config_file(void) {
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    /* Missing default file must return 0 (non-fatal fallback to defaults) */
    int rc = alps_config_load_file(&cfg, "/tmp/non_existent_alps_config_99999.conf", 0);
    assert(rc == 0);
    assert(cfg.tap.tap1_pressure == ALPS_DEFAULT_TAP1_PRESSURE);

    /* Missing explicitly specified file must return -1 (fatal error) */
    rc = alps_config_load_file(&cfg, "/tmp/non_existent_alps_config_99999.conf", 1);
    assert(rc == -1);

    printf("  [PASS] test_missing_config_file\n");
}

static void test_parse_valid_config_file(void) {
    char tmp_path[] = "/tmp/alps_test_config_XXXXXX";
    int fd = mkstemp(tmp_path);
    assert(fd >= 0);

    const char *content =
        "# ALPS Test Config\n"
        "[general]\n"
        "verbose = true\n"
        "pointer_speed = 0.25 # custom speed\n"
        "\n"
        "[tap]\n"
        "tap1_pressure = 35\n"
        "tap2_pressure = 19\n"
        "release_pressure = 8\n"
        "tap_duration_ms = 210\n"
        "tap_motion = 24\n"
        "double_tap_window_ms = 300\n"
        "drag_start_motion = 8\n"
        "drag_release_grace_ms = 180\n"
        "anti_drift = off\n"
        "\n"
        "[scroll]\n"
        "enabled = yes\n"
        "hires_multiplier = 16\n"
        "progressive = false\n"
        "accel_factor = 9\n"
        "axis_lock = 0\n";

    ssize_t written = write(fd, content, strlen(content));
    assert(written == (ssize_t)strlen(content));
    close(fd);

    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    int rc = alps_config_load_file(&cfg, tmp_path, 1);
    assert(rc == 0);

    /* Verify parsed parameters */
    assert(cfg.verbose == 1);
    assert(cfg.pointer_speed > 0.24f && cfg.pointer_speed < 0.26f);
    assert(cfg.tap.tap1_pressure == 35);
    assert(cfg.tap.tap2_rebound_pressure == 19);
    assert(cfg.tap.release_pressure == 8);
    assert(cfg.tap.max_tap_duration_ms == 210);
    assert(cfg.tap.max_tap_motion == 24);
    assert(cfg.tap.double_tap_window_ms == 300);
    assert(cfg.tap.drag_start_motion == 8);
    assert(cfg.tap.drag_release_grace_ms == 180);
    assert(cfg.tap.anti_drift_enabled == 0);

    assert(cfg.scroll.enabled == 1);
    assert(cfg.scroll.hires_multiplier == 16);
    assert(cfg.scroll.progressive_enabled == 0);
    assert(cfg.scroll.accel_factor == 9);
    assert(cfg.scroll.axis_lock_enabled == 0);

    unlink(tmp_path);
    printf("  [PASS] test_parse_valid_config_file\n");
}

static void test_config_precedence_cli_overrides_file(void) {
    char tmp_path[] = "/tmp/alps_test_precedence_XXXXXX";
    int fd = mkstemp(tmp_path);
    assert(fd >= 0);

    const char *content =
        "[tap]\n"
        "tap1_pressure = 40\n"
        "anti_drift = true\n"
        "[scroll]\n"
        "hires_multiplier = 20\n";

    ssize_t written = write(fd, content, strlen(content));
    assert(written == (ssize_t)strlen(content));
    close(fd);

    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    /* 1. Load config file (sets tap1_pressure = 40, hires = 20) */
    int rc = alps_config_load_file(&cfg, tmp_path, 1);
    assert(rc == 0);
    assert(cfg.tap.tap1_pressure == 40);
    assert(cfg.scroll.hires_multiplier == 20);
    assert(cfg.tap.anti_drift_enabled == 1);

    /* 2. CLI arguments: override tap1_pressure to 50 and disable anti_drift */
    char *argv[] = {
        "alps-stick",
        "--tap1-pressure", "50",
        "--no-anti-drift"
    };
    int argc = 4;

    int parse_rc = alps_config_parse_args(&cfg, argc, argv);
    assert(parse_rc == 0);

    /* CLI overrides took effect */
    assert(cfg.tap.tap1_pressure == 50);
    assert(cfg.tap.anti_drift_enabled == 0);
    /* Unspecified CLI flags retain file configuration */
    assert(cfg.scroll.hires_multiplier == 20);

    unlink(tmp_path);
    printf("  [PASS] test_config_precedence_cli_overrides_file\n");
}

static void test_boolean_flexibility(void) {
    char tmp_path[] = "/tmp/alps_test_bool_XXXXXX";
    int fd = mkstemp(tmp_path);
    assert(fd >= 0);

    const char *content =
        "verbose = 1\n"
        "anti_drift = FALSE\n"
        "progressive = OFF\n"
        "axis_lock = Yes\n";

    ssize_t written = write(fd, content, strlen(content));
    assert(written == (ssize_t)strlen(content));
    close(fd);

    alps_config_t cfg;
    alps_config_init_defaults(&cfg);

    int rc = alps_config_load_file(&cfg, tmp_path, 1);
    assert(rc == 0);

    assert(cfg.verbose == 1);
    assert(cfg.tap.anti_drift_enabled == 0);
    assert(cfg.scroll.progressive_enabled == 0);
    assert(cfg.scroll.axis_lock_enabled == 1);

    unlink(tmp_path);
    printf("  [PASS] test_boolean_flexibility\n");
}

static void test_config_reset_file(void) {
    char tmp_path[] = "/tmp/alps_test_reset_XXXXXX";
    int fd = mkstemp(tmp_path);
    assert(fd >= 0);

    /* Write heavily customized settings */
    const char *custom =
        "[general]\n"
        "verbose = 1\n"
        "[tap]\n"
        "tap1_pressure = 85\n"
        "tap2_pressure = 70\n"
        "release_pressure = 45\n"
        "[scroll]\n"
        "hires_multiplier = 40\n"
        "progressive = false\n";
    ssize_t written = write(fd, custom, strlen(custom));
    assert(written == (ssize_t)strlen(custom));
    close(fd);

    /* Verify custom settings load */
    alps_config_t cfg;
    alps_config_init_defaults(&cfg);
    int rc = alps_config_load_file(&cfg, tmp_path, 1);
    assert(rc == 0);
    assert(cfg.tap.tap1_pressure == 85);
    assert(cfg.scroll.hires_multiplier == 40);

    /* Execute reset to factory defaults */
    rc = alps_config_reset_file(tmp_path);
    assert(rc == 0);

    /* Reload and confirm factory defaults are restored */
    alps_config_t rst_cfg;
    alps_config_init_defaults(&rst_cfg);
    rc = alps_config_load_file(&rst_cfg, tmp_path, 1);
    assert(rc == 0);
    assert(rst_cfg.verbose == 0);
    assert(rst_cfg.tap.tap1_pressure == ALPS_DEFAULT_TAP1_PRESSURE);
    assert(rst_cfg.tap.tap2_rebound_pressure == ALPS_DEFAULT_TAP2_REBOUND_PRESSURE);
    assert(rst_cfg.tap.release_pressure == ALPS_DEFAULT_RELEASE_PRESSURE);
    assert(rst_cfg.tap.max_tap_duration_ms == ALPS_DEFAULT_MAX_TAP_DURATION_MS);
    assert(rst_cfg.tap.max_tap_motion == ALPS_DEFAULT_MAX_TAP_MOTION);
    assert(rst_cfg.tap.double_tap_window_ms == ALPS_DEFAULT_DOUBLE_TAP_WINDOW_MS);
    assert(rst_cfg.tap.anti_drift_enabled == 1);
    assert(rst_cfg.scroll.enabled == ALPS_DEFAULT_SCROLL_ENABLED);
    assert(rst_cfg.scroll.hires_multiplier == ALPS_DEFAULT_SCROLL_HIRES_MULTIPLIER);
    assert(rst_cfg.scroll.progressive_enabled == ALPS_DEFAULT_SCROLL_PROGRESSIVE);
    assert(rst_cfg.scroll.accel_factor == ALPS_DEFAULT_SCROLL_ACCEL_FACTOR);
    assert(rst_cfg.scroll.axis_lock_enabled == ALPS_DEFAULT_SCROLL_AXIS_LOCK);

    unlink(tmp_path);

    /* Test reset on non-existent file creates fresh default template */
    const char *fresh_path = "/tmp/alps_test_reset_fresh_target.conf";
    unlink(fresh_path);
    rc = alps_config_reset_file(fresh_path);
    assert(rc == 0);

    alps_config_t fresh_cfg;
    alps_config_init_defaults(&fresh_cfg);
    rc = alps_config_load_file(&fresh_cfg, fresh_path, 1);
    assert(rc == 0);
    assert(fresh_cfg.tap.tap1_pressure == ALPS_DEFAULT_TAP1_PRESSURE);
    unlink(fresh_path);

    /* Test unwritable/invalid path error handling */
    rc = alps_config_reset_file("/proc/sys/alps_invalid_target.conf");
    assert(rc == -2 || rc == -1);

    printf("  [PASS] test_config_reset_file\n");
}

int main(void) {
    printf("Running ALPS Trackstick Configuration File Unit Tests (v%s)...\n", ALPS_DAEMON_VERSION);
    test_missing_config_file();
    test_parse_valid_config_file();
    test_config_precedence_cli_overrides_file();
    test_boolean_flexibility();
    test_config_reset_file();
    printf("All 5 config file test suites passed!\n");
    return 0;
}
