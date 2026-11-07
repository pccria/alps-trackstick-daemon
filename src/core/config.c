/**
 * @file config.c
 * @brief Implementation of runtime configuration defaults and CLI flag parser.
 */

#include "core/config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <errno.h>

void alps_config_init_defaults(alps_config_t *cfg) {
    if (!cfg) return;

    cfg->verbose = 0;
    snprintf(cfg->config_path, sizeof(cfg->config_path), "%s", ALPS_DEFAULT_CONFIG_PATH);
    cfg->explicit_config = 0;
    cfg->pointer_speed = ALPS_POINTER_SPEED_UNSET;

    /* Tap / Drag feature defaults */
    cfg->tap.tap1_pressure          = ALPS_DEFAULT_TAP1_PRESSURE;
    cfg->tap.tap2_rebound_pressure  = ALPS_DEFAULT_TAP2_REBOUND_PRESSURE;
    cfg->tap.release_pressure       = ALPS_DEFAULT_RELEASE_PRESSURE;
    cfg->tap.max_tap_duration_ms    = ALPS_DEFAULT_MAX_TAP_DURATION_MS;
    cfg->tap.max_tap_motion         = ALPS_DEFAULT_MAX_TAP_MOTION;
    cfg->tap.double_tap_window_ms   = ALPS_DEFAULT_DOUBLE_TAP_WINDOW_MS;
    cfg->tap.await_motion_cancel    = ALPS_DEFAULT_AWAIT_MOTION_CANCEL;
    cfg->tap.drag_start_motion      = ALPS_DEFAULT_DRAG_START_MOTION;
    cfg->tap.drag_start_hold_ms     = ALPS_DEFAULT_DRAG_START_HOLD_MS;
    cfg->tap.drag_release_grace_ms  = ALPS_DEFAULT_DRAG_RELEASE_GRACE_MS;
    cfg->tap.click_hold_ms          = ALPS_DEFAULT_CLICK_HOLD_MS;
    cfg->tap.pre_motion_window_ms   = ALPS_DEFAULT_PRE_MOTION_WINDOW_MS;
    cfg->tap.pre_motion_limit       = ALPS_DEFAULT_PRE_MOTION_LIMIT;
    cfg->tap.motion_cap             = ALPS_DEFAULT_MOTION_CAP;
    cfg->tap.anti_drift_enabled     = 1;

    /* 2D Kinetic Scroll feature defaults */
    cfg->scroll.enabled               = ALPS_DEFAULT_SCROLL_ENABLED;
    cfg->scroll.hires_multiplier      = ALPS_DEFAULT_SCROLL_HIRES_MULTIPLIER;
    cfg->scroll.progressive_enabled   = ALPS_DEFAULT_SCROLL_PROGRESSIVE;
    cfg->scroll.accel_factor          = ALPS_DEFAULT_SCROLL_ACCEL_FACTOR;
    cfg->scroll.dominance_ratio       = ALPS_DEFAULT_SCROLL_DOMINANCE_RATIO;
    cfg->scroll.axis_lock_enabled     = ALPS_DEFAULT_SCROLL_AXIS_LOCK;
    cfg->scroll.counter_drift_enabled = ALPS_DEFAULT_SCROLL_COUNTER_DRIFT;
}

/**
 * @brief Helper to parse a bounded positive integer from a string.
 */
static int parse_int_arg(const char *arg, int min_val, int max_val, int *out_val) {
    if (!arg || !out_val) return -1;
    char *endptr = NULL;
    errno = 0;
    long val = strtol(arg, &endptr, 10);
    if (errno != 0 || endptr == arg || *endptr != '\0' || val < min_val || val > max_val) {
        return -1;
    }
    *out_val = (int)val;
    return 0;
}

static char *trim_whitespace(char *str) {
    if (!str) return NULL;
    while (isspace((unsigned char)*str)) str++;
    if (*str == '\0') return str;
    char *end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    end[1] = '\0';
    return str;
}

static int parse_bool_value(const char *val, int *out) {
    if (!val || !out) return -1;
    if (strcasecmp(val, "true") == 0 || strcasecmp(val, "yes") == 0 ||
        strcasecmp(val, "on") == 0 || strcmp(val, "1") == 0) {
        *out = 1;
        return 0;
    }
    if (strcasecmp(val, "false") == 0 || strcasecmp(val, "no") == 0 ||
        strcasecmp(val, "off") == 0 || strcmp(val, "0") == 0) {
        *out = 0;
        return 0;
    }
    return -1;
}

int alps_config_load_file(alps_config_t *cfg, const char *path, int explicit_path) {
    if (!cfg || !path) return -1;

    FILE *f = fopen(path, "r");
    if (!f) {
        if (explicit_path) {
            fprintf(stderr, "Error: Cannot open configuration file '%s': %s\n", path, strerror(errno));
            return -1;
        }
        return 0; /* Missing default configuration file is non-fatal */
    }

    char line[512];
    char section[64] = "general";

    while (fgets(line, sizeof(line), f)) {
        char *p = trim_whitespace(line);

        /* Skip comments and empty lines */
        if (*p == '\0' || *p == '#' || *p == ';') continue;

        /* Strip inline comments */
        char *comment = strpbrk(p, "#;");
        if (comment) {
            *comment = '\0';
            p = trim_whitespace(p);
            if (*p == '\0') continue;
        }

        /* Check section header: [section] */
        if (*p == '[') {
            char *end_sec = strchr(p, ']');
            if (end_sec) {
                *end_sec = '\0';
                char *sec_name = trim_whitespace(p + 1);
                snprintf(section, sizeof(section), "%s", sec_name);
                continue;
            }
        }

        /* Key = Value or Key: Value */
        char *eq = strchr(p, '=');
        if (!eq) eq = strchr(p, ':');
        if (!eq) continue;

        *eq = '\0';
        char *key = trim_whitespace(p);
        char *val = trim_whitespace(eq + 1);

        if (*key == '\0') continue;

        /* General section */
        if (strcasecmp(key, "verbose") == 0) {
            int b = 0;
            if (parse_bool_value(val, &b) == 0) cfg->verbose = b;
        } else if (strcasecmp(key, "pointer_speed") == 0 || strcasecmp(key, "speed") == 0) {
            char *endptr = NULL;
            errno = 0;
            float s = strtof(val, &endptr);
            if (errno == 0 && endptr != val && s >= -1.0f && s <= 1.0f) {
                cfg->pointer_speed = s;
            }
        }
        /* Tap section */
        else if (strcasecmp(key, "tap1_pressure") == 0) {
            int v = 0;
            if (parse_int_arg(val, 1, 255, &v) == 0) cfg->tap.tap1_pressure = v;
        } else if (strcasecmp(key, "tap2_pressure") == 0 || strcasecmp(key, "tap2_rebound_pressure") == 0) {
            int v = 0;
            if (parse_int_arg(val, 1, 255, &v) == 0) cfg->tap.tap2_rebound_pressure = v;
        } else if (strcasecmp(key, "release_pressure") == 0) {
            int v = 0;
            if (parse_int_arg(val, 0, 255, &v) == 0) cfg->tap.release_pressure = v;
        } else if (strcasecmp(key, "tap_duration_ms") == 0 || strcasecmp(key, "max_tap_duration_ms") == 0 || strcasecmp(key, "tap_duration") == 0) {
            int v = 0;
            if (parse_int_arg(val, 20, 1000, &v) == 0) cfg->tap.max_tap_duration_ms = (uint32_t)v;
        } else if (strcasecmp(key, "tap_motion") == 0 || strcasecmp(key, "max_tap_motion") == 0) {
            int v = 0;
            if (parse_int_arg(val, 1, 500, &v) == 0) cfg->tap.max_tap_motion = v;
        } else if (strcasecmp(key, "double_tap_window_ms") == 0 || strcasecmp(key, "double_window") == 0) {
            int v = 0;
            if (parse_int_arg(val, 50, 1000, &v) == 0) cfg->tap.double_tap_window_ms = (uint32_t)v;
        } else if (strcasecmp(key, "await_motion_cancel") == 0) {
            int v = 0;
            if (parse_int_arg(val, 1, 50, &v) == 0) cfg->tap.await_motion_cancel = v;
        } else if (strcasecmp(key, "drag_start_motion") == 0 || strcasecmp(key, "drag_motion") == 0) {
            int v = 0;
            if (parse_int_arg(val, 1, 200, &v) == 0) cfg->tap.drag_start_motion = v;
        } else if (strcasecmp(key, "drag_start_hold_ms") == 0) {
            int v = 0;
            if (parse_int_arg(val, 20, 1000, &v) == 0) cfg->tap.drag_start_hold_ms = (uint32_t)v;
        } else if (strcasecmp(key, "drag_release_grace_ms") == 0 || strcasecmp(key, "drag_grace") == 0 || strcasecmp(key, "drag_grace_ms") == 0) {
            int v = 0;
            if (parse_int_arg(val, 20, 2000, &v) == 0) cfg->tap.drag_release_grace_ms = (uint32_t)v;
        } else if (strcasecmp(key, "click_hold_ms") == 0) {
            int v = 0;
            if (parse_int_arg(val, 5, 500, &v) == 0) cfg->tap.click_hold_ms = (uint32_t)v;
        } else if (strcasecmp(key, "anti_drift") == 0 || strcasecmp(key, "anti_drift_enabled") == 0) {
            int b = 0;
            if (parse_bool_value(val, &b) == 0) cfg->tap.anti_drift_enabled = b;
        }
        /* Scroll section */
        else if (strcasecmp(key, "scroll_enabled") == 0 || (strcasecmp(section, "scroll") == 0 && strcasecmp(key, "enabled") == 0)) {
            int b = 0;
            if (parse_bool_value(val, &b) == 0) cfg->scroll.enabled = b;
        } else if (strcasecmp(key, "hires_multiplier") == 0 || strcasecmp(key, "scroll_speed") == 0) {
            int v = 0;
            if (parse_int_arg(val, 1, 60, &v) == 0) cfg->scroll.hires_multiplier = v;
        } else if (strcasecmp(key, "progressive") == 0 || strcasecmp(key, "progressive_enabled") == 0) {
            int b = 0;
            if (parse_bool_value(val, &b) == 0) cfg->scroll.progressive_enabled = b;
        } else if (strcasecmp(key, "accel_factor") == 0 || strcasecmp(key, "scroll_accel") == 0) {
            int v = 0;
            if (parse_int_arg(val, 0, 20, &v) == 0) cfg->scroll.accel_factor = v;
        } else if (strcasecmp(key, "axis_lock") == 0 || strcasecmp(key, "axis_lock_enabled") == 0) {
            int b = 0;
            if (parse_bool_value(val, &b) == 0) cfg->scroll.axis_lock_enabled = b;
        } else if (strcasecmp(key, "dominance_ratio") == 0) {
            int v = 0;
            if (parse_int_arg(val, 1, 10, &v) == 0) cfg->scroll.dominance_ratio = v;
        } else if (strcasecmp(key, "counter_drift") == 0 || strcasecmp(key, "counter_drift_enabled") == 0) {
            int b = 0;
            if (parse_bool_value(val, &b) == 0) cfg->scroll.counter_drift_enabled = b;
        }
    }

    fclose(f);
    return 0;
}

void alps_config_print_help(const char *prog_name) {
    printf("ALPS DualPoint Trackstick Gesture Daemon v%s\n\n", ALPS_DAEMON_VERSION);
    printf("Usage: %s [OPTIONS]\n\n", prog_name ? prog_name : "alps-stick");
    printf("General Options:\n");
    printf("  -c, --config <path>          Path to configuration file (default: %s)\n",
           ALPS_DEFAULT_CONFIG_PATH);
    printf("  -v, --verbose                Enable live gesture telemetry and state logging\n");
    printf("  -s, --speed <float>          Override libinput pointer speed [-1.0 to 1.0]\n");
    printf("                               (Default: unset — preserves native OS/KDE settings)\n");
    printf("  -h, --help                   Show this help message and exit\n");
    printf("      --version                Show version information and exit\n\n");
    printf("Transducer Calibration Options:\n");
    printf("      --tap1-pressure <int>    Tap 1 activation threshold [1..255] (default: %d)\n",
           ALPS_DEFAULT_TAP1_PRESSURE);
    printf("      --tap2-pressure <int>    Tap 2 rebound threshold [1..255] (default: %d)\n",
           ALPS_DEFAULT_TAP2_REBOUND_PRESSURE);
    printf("      --release-pressure <int> Finger unload baseline [0..255] (default: %d)\n",
           ALPS_DEFAULT_RELEASE_PRESSURE);
    printf("      --tap-duration <ms>      Max duration for an impulse tap (default: %u ms)\n",
           ALPS_DEFAULT_MAX_TAP_DURATION_MS);
    printf("      --tap-motion <counts>    Max lateral shear during a tap (default: %d)\n",
           ALPS_DEFAULT_MAX_TAP_MOTION);
    printf("      --double-window <ms>     Window between Tap 1 and Tap 2 (default: %u ms)\n",
           ALPS_DEFAULT_DOUBLE_TAP_WINDOW_MS);
    printf("      --drag-motion <counts>   Motion on Tap 2 to lock drag (default: %d)\n",
           ALPS_DEFAULT_DRAG_START_MOTION);
    printf("      --drag-grace <ms>        Inactivity grace before dropping drag (default: %u ms)\n",
           ALPS_DEFAULT_DRAG_RELEASE_GRACE_MS);
    printf("      --no-anti-drift          Disable reverse contact-shear snap on tap release\n\n");
    printf("2D Kinetic Scroll Options:\n");
    printf("      --no-scroll              Disable middle-button trackstick scrolling\n");
    printf("      --scroll-speed <mult>    Base hi-res units per count [1..60] (default: %d)\n",
           ALPS_DEFAULT_SCROLL_HIRES_MULTIPLIER);
    printf("      --linear-scroll          Disable progressive ballistics (force linear scaling)\n");
    printf("      --scroll-accel <power>   Progressive acceleration power factor [0..20] (default: %d)\n",
           ALPS_DEFAULT_SCROLL_ACCEL_FACTOR);
    printf("      --no-axis-lock           Disable cross-axis deadband (allow raw 2D diagonal)\n");
    printf("      --no-scroll-counter-drift Allow cursor to move freely during scroll\n");
}

int alps_config_parse_args(alps_config_t *cfg, int argc, char *argv[]) {
    if (!cfg) return -1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--config") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: Missing path for %s\n", argv[i]);
                return -1;
            }
            snprintf(cfg->config_path, sizeof(cfg->config_path), "%s", argv[++i]);
            cfg->explicit_config = 1;
        } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            cfg->verbose = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            alps_config_print_help(argv[0]);
            return 1;
        } else if (strcmp(argv[i], "--version") == 0) {
            printf("alps-stick v%s\n", ALPS_DAEMON_VERSION);
            return 1;
        } else if (strcmp(argv[i], "--no-anti-drift") == 0) {
            cfg->tap.anti_drift_enabled = 0;
        } else if (strcmp(argv[i], "--no-scroll") == 0) {
            cfg->scroll.enabled = 0;
        } else if (strcmp(argv[i], "--linear-scroll") == 0) {
            cfg->scroll.progressive_enabled = 0;
        } else if (strcmp(argv[i], "--no-axis-lock") == 0) {
            cfg->scroll.axis_lock_enabled = 0;
        } else if (strcmp(argv[i], "--no-scroll-counter-drift") == 0) {
            cfg->scroll.counter_drift_enabled = 0;
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--speed") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Error: Missing value for %s\n", argv[i]);
                return -1;
            }
            char *endptr = NULL;
            errno = 0;
            float val = strtof(argv[++i], &endptr);
            if (errno != 0 || endptr == argv[i] || *endptr != '\0' || val < -1.0f || val > 1.0f) {
                fprintf(stderr, "Error: Invalid speed value '%s' (must be between -1.0 and 1.0)\n", argv[i]);
                return -1;
            }
            cfg->pointer_speed = val;
        } else if (strcmp(argv[i], "--tap1-pressure") == 0) {
            if (i + 1 >= argc || parse_int_arg(argv[++i], 1, 255, &cfg->tap.tap1_pressure) < 0) {
                fprintf(stderr, "Error: Invalid --tap1-pressure value (must be 1..255)\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--tap2-pressure") == 0) {
            if (i + 1 >= argc || parse_int_arg(argv[++i], 1, 255, &cfg->tap.tap2_rebound_pressure) < 0) {
                fprintf(stderr, "Error: Invalid --tap2-pressure value (must be 1..255)\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--release-pressure") == 0) {
            if (i + 1 >= argc || parse_int_arg(argv[++i], 0, 255, &cfg->tap.release_pressure) < 0) {
                fprintf(stderr, "Error: Invalid --release-pressure value (must be 0..255)\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--tap-duration") == 0) {
            int val = 0;
            if (i + 1 >= argc || parse_int_arg(argv[++i], 20, 1000, &val) < 0) {
                fprintf(stderr, "Error: Invalid --tap-duration value (must be 20..1000 ms)\n");
                return -1;
            }
            cfg->tap.max_tap_duration_ms = (uint32_t)val;
        } else if (strcmp(argv[i], "--tap-motion") == 0) {
            if (i + 1 >= argc || parse_int_arg(argv[++i], 1, 500, &cfg->tap.max_tap_motion) < 0) {
                fprintf(stderr, "Error: Invalid --tap-motion value (must be 1..500 counts)\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--double-window") == 0) {
            int val = 0;
            if (i + 1 >= argc || parse_int_arg(argv[++i], 50, 1000, &val) < 0) {
                fprintf(stderr, "Error: Invalid --double-window value (must be 50..1000 ms)\n");
                return -1;
            }
            cfg->tap.double_tap_window_ms = (uint32_t)val;
        } else if (strcmp(argv[i], "--drag-motion") == 0) {
            if (i + 1 >= argc || parse_int_arg(argv[++i], 1, 200, &cfg->tap.drag_start_motion) < 0) {
                fprintf(stderr, "Error: Invalid --drag-motion value (must be 1..200 counts)\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--drag-grace") == 0) {
            int val = 0;
            if (i + 1 >= argc || parse_int_arg(argv[++i], 20, 2000, &val) < 0) {
                fprintf(stderr, "Error: Invalid --drag-grace value (must be 20..2000 ms)\n");
                return -1;
            }
            cfg->tap.drag_release_grace_ms = (uint32_t)val;
        } else if (strcmp(argv[i], "--scroll-speed") == 0) {
            if (i + 1 >= argc || parse_int_arg(argv[++i], 1, 60, &cfg->scroll.hires_multiplier) < 0) {
                fprintf(stderr, "Error: Invalid --scroll-speed value (must be 1..60)\n");
                return -1;
            }
        } else if (strcmp(argv[i], "--scroll-accel") == 0) {
            if (i + 1 >= argc || parse_int_arg(argv[++i], 0, 20, &cfg->scroll.accel_factor) < 0) {
                fprintf(stderr, "Error: Invalid --scroll-accel value (must be 0..20)\n");
                return -1;
            }
        } else {
            fprintf(stderr, "Error: Unknown option '%s' (see --help)\n", argv[i]);
            return -1;
        }
    }

    return 0;
}
