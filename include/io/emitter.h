/**
 * @file emitter.h
 * @brief Virtual `/dev/uinput` mouse and scroll emitter interface.
 *
 * Translates abstract `alps_action_t` descriptors from feature modules into
 * low-level Linux `uinput` frames (`EV_KEY`, `EV_REL`, `EV_SYN`).
 */

#ifndef ALPS_IO_EMITTER_H
#define ALPS_IO_EMITTER_H

#include "core/action.h"

/**
 * @brief Creates and registers the `alps-tap-emitter` virtual input device via `/dev/uinput`.
 *
 * Registers capabilities for:
 * - `EV_KEY`: `BTN_LEFT`
 * - `EV_REL`: `REL_X`, `REL_Y`, `REL_WHEEL`, `REL_HWHEEL` (and high-res wheel axes)
 *
 * @return Open `/dev/uinput` file descriptor (`>= 0`) on success, or `-1` on failure.
 */
int alps_emitter_create(void);

/**
 * @brief Destroys the virtual `uinput` device and closes its file descriptor.
 *
 * @param ufd File descriptor returned by `alps_emitter_create()`.
 */
void alps_emitter_destroy(int ufd);

/**
 * @brief Executes a high-level `alps_action_t` on the virtual `uinput` device.
 *
 * Handles anti-drift cursor snapping (`drift_x`, `drift_y` emitted in the same
 * `SYN_REPORT` frame as button events), discrete click hold durations, immediate
 * button down/up transitions, and vertical/horizontal scroll wheel events.
 *
 * @param ufd    Virtual device file descriptor.
 * @param action Pointer to the action descriptor to dispatch.
 */
void alps_emitter_dispatch(int ufd, const alps_action_t *action);

#endif /* ALPS_IO_EMITTER_H */
