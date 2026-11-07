# ALPS DualPoint Trackstick Gesture Daemon

A lightweight, modular native Linux userspace daemon that restores tap-to-click, double-click, and tap-and-drag gestures to ALPS DualPoint pointing sticks by intercepting raw kernel `evdev` pressure streams and synthesizing mouse/scroll events via `uinput`.

## Background

Under modern Linux desktop environments (Wayland/X11 with `libinput`), tracksticks are handled purely as relative motion pointers. The kernel's `alps.c` driver reports downward physical force on the stick's rubber cap as `ABS_PRESSURE`, but `libinput` disregards pressure on pointing sticks because tap gestures are only defined for touchpads.

This daemon runs in the background, connects to the raw `/dev/input/event*` node of the ALPS pointing stick, evaluates contact dynamics through a pure, unit-tested finite state machine, and emits clean `BTN_LEFT` clicks and anti-drift corrections via `/dev/uinput` with zero CPU busy-polling.

## Project Structure (`v0.5.0` Event-Action Pipeline)

The codebase is organized into a clean, extensible **Event-Action Pipeline** separating pure gesture physics from Linux kernel `evdev`/`uinput` plumbing:

```text
alps-trackstick-daemon/
├── include/
│   ├── core/
│   │   ├── event.h          # Normalized input event struct & saturating math helpers
│   │   ├── action.h         # Abstract output actions (Click, Down/Up, Anti-Drift, Scroll)
│   │   └── config.h         # Runtime configuration struct, calibrated defaults, CLI parser
│   ├── features/
│   │   └── tap_gesture.h    # Pure, zero-I/O tap, double-click, and drag state machine API
│   └── io/
│       ├── device.h         # /dev/input discovery & optional xinput speed helper
│       └── emitter.h        # Virtual /dev/uinput mouse & wheel action dispatcher
├── src/
│   ├── main.c               # Daemon entry point, signal handling, and epoll event loop
│   ├── core/
│   │   └── config.c         # Default parameters and command-line argument parser
│   ├── features/
│   │   └── tap_gesture.c    # Anti-drift physical transducer state machine implementation
│   └── io/
│       ├── device.c         # EVIOCGNAME scanning and safe fork/execv xinput helper
│       └── emitter.c        # uinput device lifecycle and frame emission
├── tests/
│   └── test_tap_gesture.c   # Deterministic, root-free unit test suite
├── Makefile
└── alps-stick.service
```

## Architecture: Anti-Drift Physical Transducer

Rather than attempting to guess user intent through delayed confirmation heuristics, the daemon implements an **Anti-Drift Physical Transducer**:

1. **Light, Smooth Tap Activation ($Z \ge 22$):**
   * Keeps the contact floor at $Z=22$, enabling gentle, effortless, smooth tapping.
   * **Force-Ratio Discriminator:** Requires $Z \ge \text{motion} \times 1.5$ for any lateral movement $> 6$ counts, distinguishing true downward taps from steering micro-nudges.

2. **Anti-Drift Snap (Zero Target Slip):**
   * When an index finger strikes a flexible rubber dome, contact shear physically deflects the pointer by a few pixels ($dx, dy$).
   * Upon lift-off, the daemon emits an equal and opposite relative motion pulse ($-dx, -dy$) via `uinput` in the **exact same `SYN_REPORT` frame** as the click.
   * The cursor snaps right back to the pre-tap coordinate before the click lands, completely eliminating cursor slip.

3. **Immediate Virtual Button Down on Second Contact (Tap 2):**
   * Sensitive rebound threshold ($Z \ge 15$) captures natural second strikes without false triggers.
   * Emits `BTN_LEFT 1` **immediately on touchdown**.
   * **Double-Click:** If the finger lifts off within 180ms without lateral motion ($< 6$ counts), drift is reversed and `BTN_LEFT 0` fires immediately. The desktop compositor registers a native, lag-free double click.
   * **Tap-and-Drag:** If the stick is steered laterally ($\ge 6$ counts) or held stationary ($\ge 160\text{ms}$), drag lock engages instantly from millisecond zero with zero grab hesitation.

4. **Motion-Coupled Drag Retention & Crisp Drop:**
   * Steering tilts the stick sideways, which naturally unloads downward vertical pressure $Z$ to near zero.
   * Active relative motion (`EV_REL`) sustains the drag lock continuously.
   * A tuned 140ms inactivity grace window eliminates sticky drag linger while protecting against accidental drops during direction reversals.

5. **Steering Misclick & Drag-Collision Protection:**
   * Decaying velocity memory tracks active pointer navigation.
   * If cursor movement begins during the Tap 2 primed window, just 3 counts of movement immediately aborts the primed state, ensuring a seamless transition from clicking to steering without accidental drags.
   * Native KDE/desktop pointer speeds and acceleration curves remain 100% untampered by default.

## Installation & Testing

### Prerequisites

```bash
sudo apt install build-essential gcc
```

### Build & Run Unit Tests

The transducer state machine can be verified deterministically without root privileges or hardware access:

```bash
make
make test
```

### Install Systemd Service

```bash
sudo make install
```

This compiles `bin/alps-stick`, installs the binary to `/usr/local/bin/`, sets up the systemd service at `/etc/systemd/system/alps-stick.service`, and enables it to run on boot.

### Runtime CLI Options

All physical thresholds can be inspected or tuned at runtime without recompiling:

```text
Usage: ./bin/alps-stick [OPTIONS]

General Options:
  -v, --verbose                Enable live gesture telemetry and state logging
  -s, --speed <float>          Override libinput pointer speed [-1.0 to 1.0]
                               (Default: unset — preserves native OS/KDE settings)
  -h, --help                   Show this help message and exit
      --version                Show version information and exit

Transducer Calibration Options:
      --tap1-pressure <int>    Tap 1 activation threshold [1..255] (default: 22)
      --tap2-pressure <int>    Tap 2 rebound threshold [1..255] (default: 15)
      --release-pressure <int> Finger unload baseline [0..255] (default: 10)
      --tap-duration <ms>      Max duration for an impulse tap (default: 180 ms)
      --tap-motion <counts>    Max lateral shear during a tap (default: 16)
      --double-window <ms>     Window between Tap 1 and Tap 2 (default: 260 ms)
      --drag-motion <counts>   Motion on Tap 2 to lock drag (default: 6)
      --drag-grace <ms>        Inactivity grace before dropping drag (default: 140 ms)
      --no-anti-drift          Disable reverse contact-shear snap on tap release
```

### Service Management

```bash
# Check service status
systemctl status alps-stick.service

# View live daemon logs
journalctl -u alps-stick.service -f
```

## Uninstallation

```bash
sudo make uninstall
```

## License

MIT
