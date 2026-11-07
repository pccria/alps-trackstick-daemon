/**
 * @file config.h
 * @brief Runtime configuration structures, calibrated defaults, and CLI parser API.
 *
 * Centralizes all physical transducer thresholds and scroll parameters so they
 * can be inspected, tuned at runtime via command-line flags without recompiling,
 * or passed into unit tests with custom synthetic parameters.
 */

#ifndef ALPS_CORE_CONFIG_H
#define ALPS_CORE_CONFIG_H

#include <stdint.h>

/** @brief Daemon semantic version string. */
#define ALPS_DAEMON_VERSION                  "0.7.0"

/* --- Calibrated Default Transducer Thresholds --- */
#define ALPS_DEFAULT_TAP1_PRESSURE           22    /**< Smooth, feather-light activation floor for Tap 1 */
#define ALPS_DEFAULT_TAP2_REBOUND_PRESSURE   15    /**< Sensitive rebound floor for natural low-amplitude Tap 2 */
#define ALPS_DEFAULT_RELEASE_PRESSURE        10    /**< Mechanical unload baseline */
#define ALPS_DEFAULT_MAX_TAP_DURATION_MS     180   /**< Maximum contact duration (ms) for an impulse tap */
#define ALPS_DEFAULT_MAX_TAP_MOTION          16    /**< Maximum allowable lateral contact shear counts */
#define ALPS_DEFAULT_DOUBLE_TAP_WINDOW_MS    260   /**< Primed window (ms) between Tap 1 release and Tap 2 contact */
#define ALPS_DEFAULT_AWAIT_MOTION_CANCEL     3     /**< Motion counts that abort primed state on first steering packet */
#define ALPS_DEFAULT_DRAG_START_MOTION       6     /**< Lateral motion counts on Tap 2 that lock drag immediately */
#define ALPS_DEFAULT_DRAG_START_HOLD_MS      160   /**< Stationary hold time (ms) on Tap 2 to lock drag */
#define ALPS_DEFAULT_DRAG_RELEASE_GRACE_MS   140   /**< Inactivity grace timeout (ms) before dropping drag */
#define ALPS_DEFAULT_CLICK_HOLD_MS           15    /**< Minimum button-down hold duration (ms) for discrete clicks */
#define ALPS_DEFAULT_PRE_MOTION_WINDOW_MS    150   /**< Velocity memory window (ms) to detect active navigation */
#define ALPS_DEFAULT_PRE_MOTION_LIMIT        10    /**< Accumulated motion in window that classifies spikes as steering */
#define ALPS_DEFAULT_MOTION_CAP              10000 /**< Saturation ceiling for motion accumulators */
#define ALPS_POINTER_SPEED_UNSET             (-999.0f) /**< Sentinel indicating native OS ballistics are preserved */

/* --- Calibrated Default Scroll Parameters --- */
#define ALPS_DEFAULT_SCROLL_ENABLED          1     /**< Middle-button scrolling enabled by default */
#define ALPS_DEFAULT_SCROLL_HIRES_MULTIPLIER 15    /**< High-res units per deflection count (8 counts = 1 notch = 120 units) */
#define ALPS_DEFAULT_SCROLL_DOMINANCE_RATIO  2     /**< Axis dominance ratio (2:1) for vector cone tremor rejection */
#define ALPS_DEFAULT_SCROLL_AXIS_LOCK        1     /**< Vector cone deadband enabled by default to prevent lateral drift */
#define ALPS_DEFAULT_SCROLL_COUNTER_DRIFT    1     /**< Snap cursor back with (-dx, -dy) during scroll frames */

/**
 * @brief Tunable parameters for the tap, double-click, and drag transducer.
 */
typedef struct {
    int tap1_pressure;               /**< Activation pressure threshold for initial tap (Z >= tap1_pressure) */
    int tap2_rebound_pressure;       /**< Activation pressure threshold for rebound tap (Z >= tap2_rebound_pressure) */
    int release_pressure;            /**< Unload threshold where finger is considered lifted (Z <= release_pressure) */
    uint32_t max_tap_duration_ms;    /**< Maximum contact duration in ms for a discrete tap */
    int max_tap_motion;              /**< Maximum lateral shear counts permitted during a tap */
    uint32_t double_tap_window_ms;   /**< Time window in ms after Tap 1 release to accept Tap 2 */
    int await_motion_cancel;         /**< Motion threshold during primed window that cancels into navigation */
    int drag_start_motion;           /**< Motion threshold during Tap 2 that locks into active dragging */
    uint32_t drag_start_hold_ms;     /**< Stationary hold duration in ms during Tap 2 that locks into dragging */
    uint32_t drag_release_grace_ms;  /**< Inactivity grace window in ms before releasing an active drag */
    uint32_t click_hold_ms;          /**< Hold duration in ms for synthesized discrete clicks */
    uint32_t pre_motion_window_ms;   /**< Rolling window in ms for pre-press navigation velocity tracking */
    int pre_motion_limit;            /**< Motion count threshold in pre-motion window that rejects taps */
    int motion_cap;                  /**< Maximum saturation cap for motion accumulators */
    int anti_drift_enabled;          /**< Non-zero to snap cursor back by (-dx, -dy) upon tap release */
} alps_tap_config_t;

/**
 * @brief Tunable parameters for 2D kinetic trackstick scrolling.
 */
typedef struct {
    int enabled;                     /**< 1 to enable middle-button scrolling, 0 to disable */
    int hires_multiplier;            /**< High-res scroll units per motion count (default: 15) */
    int dominance_ratio;             /**< Dominance ratio (e.g. 2 for 2:1) for vector cone filtering */
    int axis_lock_enabled;           /**< 1 to enforce transient vector cone filtering, 0 for raw unconstrained 2D */
    int counter_drift_enabled;       /**< 1 to counteract cursor displacement with (-dx, -dy) during scroll */
} alps_scroll_config_t;

/**
 * @brief Top-level daemon configuration container.
 */
typedef struct {
    int verbose;                     /**< Enable live gesture telemetry and state transition logging */
    float pointer_speed;             /**< Optional xinput accel speed [-1.0, 1.0], or ALPS_POINTER_SPEED_UNSET */
    alps_tap_config_t tap;           /**< Tap, double-click, and drag feature configuration */
    alps_scroll_config_t scroll;     /**< 2D kinetic scrolling feature configuration */
} alps_config_t;

/**
 * @brief Populates `cfg` with empirically calibrated default parameters.
 *
 * @param cfg Pointer to configuration structure to initialize.
 */
void alps_config_init_defaults(alps_config_t *cfg);

/**
 * @brief Parses command-line arguments into `cfg`.
 *
 * @param cfg  Pointer to initialized configuration structure.
 * @param argc Argument count from `main()`.
 * @param argv Argument vector from `main()`.
 * @return `0` on success, `1` if `--help` or `--version` was handled (caller should exit 0),
 *         or `-1` on invalid arguments (caller should exit non-zero).
 */
int alps_config_parse_args(alps_config_t *cfg, int argc, char *argv[]);

/**
 * @brief Prints CLI usage instructions and default parameter values to `stdout`.
 *
 * @param prog_name Executable name (`argv[0]`).
 */
void alps_config_print_help(const char *prog_name);

#endif /* ALPS_CORE_CONFIG_H */
