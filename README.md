# ALPS DualPoint Trackstick Gesture Daemon

A lightweight native Linux userspace daemon that restores tap-to-click, double-click, and tap-and-drag gestures to ALPS DualPoint pointing sticks by intercepting raw kernel `evdev` pressure streams and synthesizing mouse events via `uinput`.

## Background

Under modern Linux desktop environments (Wayland/X11 with `libinput`), tracksticks are handled purely as relative motion pointers. The kernel's `alps.c` driver reports downward physical force on the stick's rubber cap as `ABS_PRESSURE`, but `libinput` disregards pressure on pointing sticks because tap gestures are only defined for touchpads.

This daemon runs in the background, connects to the raw `/dev/input/event*` node of the ALPS pointing stick, tracks contact dynamics through a low-latency state machine, and emits clean `BTN_LEFT` clicks via `/dev/uinput` with zero CPU busy-polling.

## Architecture: Physical Virtual Transducer

Rather than attempting to guess user intent through delayed confirmation heuristics, `v0.3.0` implements a zero-latency **Physical Virtual Transducer**:

1. **Lift-Off Gated Single Click (Tap 1):**
   * Contact trigger: downward pressure $Z \ge 22$.
   * Release: $Z \le 10$ within 180ms with lateral shear $\le 30$ counts.
   * Emits a clean discrete `BTN_LEFT` pulse (15ms hold), perfectly registered by X11/Wayland event loops.
   * Primes a 300ms window for Tap 2.

2. **Immediate Virtual Button Down on Second Contact (Tap 2):**
   * Sensitive rebound threshold ($Z \ge 14$) captures natural low-amplitude second strikes.
   * Emits `BTN_LEFT 1` **immediately on touchdown**.
   * **Double-Click:** If the finger lifts off within 180ms without lateral motion, `BTN_LEFT 0` fires immediately. The desktop compositor registers a native, lag-free double click.
   * **Tap-and-Drag:** If the stick is steered laterally ($\ge 15$ counts) or held stationary ($\ge 180\text{ms}$), drag lock engages instantly from millisecond zero.

3. **Motion-Coupled Drag Retention:**
   * Steering tilts the stick sideways, which naturally drops downward vertical pressure $Z$ to near zero.
   * Active relative motion (`EV_REL`) sustains the drag lock continuously.
   * Generous 250ms inactivity grace window prevents accidental drops during direction reversals and micro-pauses.

4. **Steering Misclick Protection:**
   * Rolling 80ms velocity window detects active cursor navigation ($\ge 15$ counts).
   * Downward pressure spikes during rapid navigation are classified as steering force and ignored.
   * If cursor movement begins during the 300ms Tap 2 primed window, the primed state is canceled immediately, ensuring normal navigation remains unhindered.

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

Run the binary directly from a terminal to inspect live gesture diagnostics and sensor telemetry:

```bash
sudo ./bin/alps-stick -v
```

## Uninstallation

```bash
sudo make uninstall
```

## License

MIT
