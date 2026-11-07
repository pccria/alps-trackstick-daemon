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
        "axis_lock = 0\n"
        "counter_drift = on\n";

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
    assert(cfg.scroll.counter_drift_enabled == 1);

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

int main(void) {
    printf("Running ALPS Trackstick Configuration File Unit Tests (v%s)...\n", ALPS_DAEMON_VERSION);
    test_missing_config_file();
    test_parse_valid_config_file();
    test_config_precedence_cli_overrides_file();
    test_boolean_flexibility();
    printf("All 4 config file test suites passed!\n");
    return 0;
}
