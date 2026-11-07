/**
 * @file calibrate.h
 * @brief Interactive terminal calibration wizard and transducer force telemetry.
 *
 * Provides a live ASCII visualizer of strain-gauge pressure, peak contact
 * force, lateral steering shear, and tap duration. Computes tailored
 * threshold recommendations for `/etc/alps-stick.conf`.
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
 * @brief Initializes the calibration tracker to default zero values.
 */
void alps_calibrate_tracker_init(alps_calibrate_tracker_t *tracker);

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
 * @brief Derives recommended configuration thresholds based on observed physical tap data.
 *
 * @param tracker     Tracker containing recorded tap sessions.
 * @param out_tap1    Output recommended tap1_pressure.
 * @param out_tap2    Output recommended tap2_rebound_pressure.
 * @param out_release Output recommended release_pressure.
 */
void alps_calibrate_compute_recommendation(const alps_calibrate_tracker_t *tracker,
                                          int *out_tap1,
                                          int *out_tap2,
                                          int *out_release);

/**
 * @brief Runs the interactive full-terminal calibration loop.
 *
 * @param cfg Daemon configuration container.
 * @return 0 on clean exit, non-zero on error.
 */
int alps_calibrate_run(const alps_config_t *cfg);

#endif /* ALPS_FEATURES_CALIBRATE_H */
