/**
 * @file calibrate.h
 * @brief Interactive terminal calibration wizard and transducer force telemetry.
 *
 * Provides a guided 4-phase physical profiler (resting noise, single taps,
 * double-click rebound, and steering shear), a statistical metrics dashboard,
 * and automated `/etc/alps-stick.conf` configuration updating.
 */

#ifndef ALPS_FEATURES_CALIBRATE_H
#define ALPS_FEATURES_CALIBRATE_H

#include <stdint.h>
#include <stddef.h>
#include "core/config.h"
#include "core/event.h"

/**
 * @brief Live physical contact and tap duration tracking state.
 */
typedef struct {
    int current_z;                   /**< Instantaneous transducer pressure Z [0..255] */
    int current_dx;                  /**< Instantaneous frame horizontal shear dX */
    int current_dy;                  /**< Instantaneous frame vertical shear dY */
    int max_tap_z;                   /**< Peak pressure registered across all recorded taps */
    int min_tap_z;                   /**< Minimum peak pressure registered on valid taps */
    int tap_count;                   /**< Number of valid impulse taps completed */
    uint64_t tap_peak_sum;           /**< Sum of peak pressures for average calculation */
    uint64_t tap_duration_sum;       /**< Sum of contact durations (ms) for average calculation */
    uint64_t active_press_start_ms;  /**< Timestamp of current contact start, or 0 if released */
    int active_peak_z;               /**< Peak pressure reached during current press window */
    int active_motion_counts;        /**< Accumulated shear motion during current contact window */
} alps_calibrate_tracker_t;

/**
 * @brief Multi-phase physical profile gathered across the guided calibration wizard.
 */
typedef struct {
    /* Phase 1: Resting baseline & noise floor */
    int noise_floor_z;               /**< Average resting pressure reading while untouched */
    int noise_floor_max_z;           /**< Maximum transient noise spike observed while resting */

    /* Phase 2: Single impulse taps */
    int single_tap_count;            /**< Number of single taps recorded */
    int single_tap_avg_peak;         /**< Average peak strike pressure for single taps */
    int single_tap_min_peak;         /**< Lightest recorded single tap peak */
    int single_tap_max_peak;         /**< Heaviest recorded single tap peak */
    uint32_t single_tap_avg_dur_ms;  /**< Average contact dwell time in milliseconds */

    /* Phase 3: Double-click cadence & rebound */
    int double_tap_count;            /**< Number of double-tap pairs recorded */
    int double_tap1_avg_peak;        /**< Average peak pressure of initial tap */
    int double_tap2_avg_peak;        /**< Average peak pressure of rebound tap */
    uint32_t double_tap_avg_gap_ms;  /**< Average inter-tap pause duration (ms) */

    /* Phase 4: Navigation steering shear */
    int steer_max_z;                 /**< Maximum vertical force during active steering */
    int steer_max_motion;            /**< Maximum frame shear observed during active navigation */

    /* Derived optimal parameters */
    int rec_tap1_pressure;           /**< Recommended tap1_pressure */
    int rec_tap2_pressure;           /**< Recommended tap2_rebound_pressure */
    int rec_release_pressure;        /**< Recommended release_pressure */
    uint32_t rec_tap_duration_ms;    /**< Recommended max_tap_duration_ms */
    uint32_t rec_double_window_ms;   /**< Recommended double_tap_window_ms */
    int rec_tap_motion;              /**< Recommended max_tap_motion */
} alps_guided_profile_t;

/**
 * @brief Initializes the calibration tracker to default zero values.
 */
void alps_calibrate_tracker_init(alps_calibrate_tracker_t *tracker);

/**
 * @brief Initializes a guided physical profile container.
 */
void alps_calibrate_profile_init(alps_guided_profile_t *prof);

/**
 * @brief Updates tracker telemetry with a newly received hardware event.
 *
 * @param tracker  Pointer to calibration tracker.
 * @param ev       Hardware event packet (pressure, motion, syn).
 * @param now_ms   Monotonic timestamp in milliseconds.
 * @return 1 if a tap completed on this event (released after contact), 0 otherwise.
 */
int alps_calibrate_tracker_feed_event(alps_calibrate_tracker_t *tracker,
                                      const alps_event_t *ev,
                                      uint64_t now_ms);

/**
 * @brief Formats an ASCII horizontal bar representing current pressure.
 *
 * Example: `[=======>                 ]`
 *
 * @param z       Current pressure value.
 * @param max_z   Full-scale pressure value (e.g., 100 or 255).
 * @param width   Character width of the bar interior.
 * @param out_buf Buffer to receive formatted string.
 * @param out_max Maximum bytes to write to out_buf.
 */
void alps_calibrate_format_bar(int z, int max_z, int width, char *out_buf, size_t out_max);

/**
 * @brief Derives basic recommendations from an open-ended tracker session.
 */
void alps_calibrate_compute_recommendation(const alps_calibrate_tracker_t *tracker,
                                          int *out_tap1,
                                          int *out_tap2,
                                          int *out_release);

/**
 * @brief Derives comprehensive optimal parameters from a guided 4-phase physical profile.
 *
 * @param prof Profile containing measured physical metrics from all 4 phases.
 */
void alps_calibrate_compute_guided_recommendations(alps_guided_profile_t *prof);

/**
 * @brief Merges and saves a calibrated physical profile into a configuration file.
 *
 * Preserves user comments, `[general]`, and `[scroll]` sections while updating
 * or creating the `[tap]` calibration block.
 *
 * @param config_path Destination configuration file (e.g. `/etc/alps-stick.conf`).
 * @param prof        Profile containing derived recommended parameters.
 * @return 0 on success, -1 on I/O error, -2 on permission denied.
 */
int alps_calibrate_save_config(const char *config_path, const alps_guided_profile_t *prof);

/**
 * @brief Runs the guided interactive terminal calibration wizard.
 *
 * @param cfg Daemon configuration container.
 * @return 0 on clean exit, non-zero on error.
 */
int alps_calibrate_run(const alps_config_t *cfg);

/**
 * @brief Runs the live diagnostic telemetry HUD showing instantaneous pressure, shear, and states.
 *
 * @param cfg Daemon configuration container.
 * @return 0 on clean exit, non-zero on error.
 */
int alps_calibrate_run_hud(const alps_config_t *cfg);

/**
 * @brief Launches the unified interactive terminal Control Center.
 *
 * @param cfg Daemon configuration container.
 * @return 0 on clean exit, non-zero on error.
 */
int alps_interactive_menu_run(const alps_config_t *cfg);

#endif /* ALPS_FEATURES_CALIBRATE_H */
