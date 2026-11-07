/**
 * @file device.h
 * @brief ALPS DualPoint Stick `/dev/input` discovery and optional `xinput` speed helper.
 *
 * Isolates Linux `evdev` directory scanning, kernel `EVIOCGNAME` ioctl probing,
 * and subprocess execution (`fork`/`execv`) away from the gesture state machines.
 */

#ifndef ALPS_IO_DEVICE_H
#define ALPS_IO_DEVICE_H

#include <stddef.h>

/**
 * @brief Scans `/dev/input/event*` to locate the ALPS DualPoint Stick node.
 *
 * Opens each candidate character device in non-blocking read-only mode (`O_RDONLY | O_NONBLOCK | O_CLOEXEC`)
 * and queries `EVIOCGNAME` for `"DualPoint Stick"`.
 *
 * @param out_path Buffer to receive the matched device path (e.g., `"/dev/input/event10"`).
 * @param path_max Size of `out_path` buffer in bytes.
 * @param out_name Buffer to receive the matched kernel device name.
 * @param name_max Size of `out_name` buffer in bytes.
 * @return Open non-blocking file descriptor (`>= 0`) on success, or `-1` if not found.
 */
int alps_device_find_stick(char *out_path, size_t path_max,
                           char *out_name, size_t name_max);

/**
 * @brief Optionally applies a `libinput Accel Speed` override via `/usr/bin/xinput`.
 *
 * By default, the daemon leaves `pointer_speed` set to `ALPS_POINTER_SPEED_UNSET`
 * so desktop environment settings (such as KDE Plasma Flat profile acceleration)
 * remain untouched. If the user explicitly passes `-s` / `--speed` in `[-1.0, 1.0]`,
 * this function invokes `/usr/bin/xinput` directly via `fork()` + `execv()`
 * without invoking `/bin/sh`.
 *
 * @param dev_name Kernel device name reported by `EVIOCGNAME`.
 * @param speed    Desired libinput acceleration speed in `[-1.0, 1.0]`.
 */
void alps_device_apply_pointer_speed(const char *dev_name, float speed);

#endif /* ALPS_IO_DEVICE_H */
