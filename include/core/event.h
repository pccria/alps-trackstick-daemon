/**
 * @file event.h
 * @brief Normalized hardware-independent input event definitions and safe math helpers.
 *
 * Decouples feature state machines (tap, drag, scroll) from raw Linux kernel
 * `struct input_event` types so gesture logic can be evaluated deterministically
 * and unit-tested without `/dev/input` file descriptors.
 */

#ifndef ALPS_CORE_EVENT_H
#define ALPS_CORE_EVENT_H

#include <stdint.h>
#include <limits.h>

/**
 * @brief Normalized input axis/sensor event categories.
 */
typedef enum {
    ALPS_EV_REL_X = 0,   /**< Lateral horizontal deflection (X-axis strain) */
    ALPS_EV_REL_Y,       /**< Lateral vertical deflection (Y-axis strain) */
    ALPS_EV_PRESSURE,    /**< Vertical downward force on the cap (Z-axis pressure) */
    ALPS_EV_BTN_MIDDLE,  /**< Physical trackstick middle button state (1=down, 0=up) */
    ALPS_EV_SYN_REPORT   /**< Hardware event frame synchronization boundary */
} alps_event_type_t;

/**
 * @brief A single timestamped sensor reading fed into the feature pipeline.
 */
typedef struct {
    alps_event_type_t type; /**< Sensor axis type */
    int32_t value;          /**< Raw axis delta or absolute pressure reading */
    uint64_t timestamp_ms;  /**< Monotonic clock timestamp in milliseconds */
} alps_event_t;

/**
 * @brief Safely accumulates absolute motion magnitude with saturation.
 *
 * Prevents two C undefined behavior traps on untrusted/noisy hardware streams:
 * 1. `abs(INT32_MIN)` signed overflow UB.
 * 2. Signed integer wrap-around during prolonged high-speed movement.
 *
 * @param current Current accumulated motion magnitude (must be >= 0).
 * @param delta   Signed relative motion delta from the sensor.
 * @param cap     Maximum saturation ceiling (e.g., 10000).
 * @return Saturated motion sum in the range `[0, cap]`.
 */
static inline int alps_add_motion_saturated(int current, int32_t delta, int cap) {
    int32_t mag;
    if (delta == INT32_MIN) {
        mag = cap;
    } else {
        mag = delta < 0 ? -delta : delta;
    }
    if (mag > cap) {
        mag = cap;
    }
    if (current > cap - (int)mag) {
        return cap;
    }
    return current + (int)mag;
}

#endif /* ALPS_CORE_EVENT_H */
