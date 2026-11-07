# ALPS DualPoint Trackstick Gesture Daemon

A lightweight native Linux userspace daemon that restores tap-to-click, double-click, and tap-and-drag gestures to ALPS DualPoint pointing sticks by intercepting raw kernel `evdev` pressure streams and synthesizing mouse events via `uinput`.

## Background

Under modern Linux desktop environments (Wayland/X11 with `libinput`), tracksticks are handled purely as relative motion pointers. The kernel's `alps.c` driver reports downward physical force on the stick's rubber cap as `ABS_PRESSURE`, but `libinput` disregards pressure on pointing sticks because tap gestures are only defined for touchpads.

This daemon runs in the background, connects to the raw `/dev/input/event*` node of the ALPS pointing stick, tracks contact dynamics through a low-latency state machine, and emits clean `BTN_LEFT` clicks via `/dev/uinput` with zero CPU busy-polling.

## Features

* **Lift-Off Gated Single Click:** Fires on release rather than touchdown, preventing pointer wobble and target slip.
* **Immediate Double-Tap Drag Lock:** Locks the virtual button down on the instant of the second tap for lag-free text selection and window dragging.
* **Dual-Gated Drag Retention:** Bridges lateral steering shear dropouts using motion detection while keeping drag held as long as downward pressure remains.
* **Zero Overhead:** Native C implementation using Linux `epoll` with dynamic timeouts (~25 KB ELF binary, ~400 KB RSS memory footprint).
* **Systemd Integration:** Clean service unit that automatically starts on boot when hardware input initializes.

## Installation

### Prerequisites

```bash
sudo apt install build-essential gcc
```

### Build & Install

```bash
make
sudo make install
```

This compiles `bin/alps-stick`, installs the binary to `/usr/local/bin/`, sets up the systemd service at `/etc/systemd/system/alps-stick.service`, and enables it to run on boot.

### Service Management

```bash
# Check service status
systemctl status alps-stick.service

# View live daemon logs
journalctl -u alps-stick.service -f
```

### Manual Testing

You can run the binary directly from terminal to see live gesture diagnostics:

```bash
sudo ./bin/alps-stick -v
```

## Uninstallation

```bash
sudo make uninstall
```

## License

MIT
