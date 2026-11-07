# ALPS DualPoint Trackstick Gesture Daemon

A lightweight native Linux userspace daemon that restores tap-to-click, double-click, and tap-and-drag gestures to ALPS DualPoint pointing sticks by intercepting raw kernel `evdev` pressure streams and synthesizing mouse events via `uinput`.

## Background

Under modern Linux desktop environments (Wayland/X11 with `libinput`), tracksticks are handled purely as relative motion pointers. The kernel's `alps.c` driver reports downward physical force on the stick's rubber cap as `ABS_PRESSURE`, but `libinput` disregards pressure on pointing sticks because tap gestures are only defined for touchpads.

This daemon runs in the background, connects to the raw `/dev/input/event*` node of the ALPS pointing stick, tracks contact dynamics through a low-latency state machine, and emits clean `BTN_LEFT` clicks via `/dev/uinput` with zero CPU busy-polling.

## Architecture: Anti-Drift Physical Transducer

Rather than attempting to guess user intent through delayed confirmation heuristics, `v0.4.0` implements an **Anti-Drift Physical Transducer**:

1. **Light, Smooth Tap Activation ($Z \ge 22$):**
   * Lowers the contact floor back down to $Z=22$, restoring gentle, effortless, smooth tapping (no hard whacking required).
   * **Physical Discriminator:** Requires $Z \ge \text{motion} \times 1.5$ for any lateral movement $> 6$ counts, distinguishing true downward taps from steering micro-nudges.

2. **Anti-Drift Snap (Zero Target Slip):**
   * When an index finger strikes a flexible rubber dome, contact shear physically deflects the pointer by a few pixels ($dx, dy$).
   * Upon lift-off, the daemon emits an equal and opposite relative motion pulse ($-dx, -dy$) via `uinput` in the **exact same event frame** as the click.
   * The cursor snaps right back to the pre-tap coordinate before the click lands, completely eliminating cursor slip!

3. **Immediate Virtual Button Down on Second Contact (Tap 2):**
   * Sensitive rebound threshold ($Z \ge 15$) captures natural second strikes without false triggers.
   * Emits `BTN_LEFT 1` **immediately on touchdown**.
   * **Double-Click:** If the finger lifts off within 180ms without lateral motion ($< 6$ counts), drift is reversed and `BTN_LEFT 0` fires immediately. The desktop compositor registers a native, lag-free double click.
   * **Tap-and-Drag:** If the stick is steered laterally ($\ge 6$ counts) or held stationary ($\ge 160\text{ms}$), drag lock engages instantly from millisecond zero with zero grab hesitation.

4. **Motion-Coupled Drag Retention & Crisp Drop:**
   * Steering tilts the stick sideways, which naturally drops downward vertical pressure $Z$ to near zero.
   * Active relative motion (`EV_REL`) sustains the drag lock continuously.
   * Tuned 140ms inactivity grace window eliminates sticky drag linger while fully protecting against accidental drops during direction reversals.

5. **Steering Misclick & Drag-Collision Protection:**
   * Decaying velocity memory tracks active pointer navigation.
   * If cursor movement begins during the Tap 2 primed window, just 3 counts of movement immediately aborts the primed state, ensuring seamless transition from clicking to steering without accidental drags.
   * Native KDE/desktop pointer speeds and acceleration curves remain 100% untampered.

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
