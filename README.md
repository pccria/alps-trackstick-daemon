# ALPS DualPoint Trackstick Gesture Daemon

A lightweight, modular native Linux userspace daemon that restores tap-to-click, double-click, tap-and-drag gestures, and high-resolution 2D kinetic scrolling to ALPS DualPoint pointing sticks by intercepting raw kernel `evdev` pressure/button streams and synthesizing mouse/scroll events via `uinput`.

## Background

Under modern Linux desktop environments (Wayland/X11 with `libinput`), tracksticks are handled purely as relative motion pointers. The kernel's `alps.c` driver reports downward physical force on the stick's rubber cap as `ABS_PRESSURE`, but `libinput` disregards pressure on pointing sticks because tap gestures are only defined for touchpads.

This daemon runs in the background, connects to the raw `/dev/input/event*` node of the ALPS pointing stick, evaluates contact dynamics through pure, unit-tested finite state machines, and emits clean `BTN_LEFT` clicks, anti-drift corrections, and continuous 2D wheel events via `/dev/uinput` with zero CPU busy-polling.

## Project Structure (`v0.7.0` Event-Action Pipeline)

The codebase is organized into a clean, extensible **Event-Action Pipeline** separating pure physics and wheel math from Linux kernel `evdev`/`uinput` plumbing:

```text
alps-trackstick-daemon/
├── include/
│   ├── core/
│   │   ├── event.h          # Normalized input event struct & saturating math helpers
│   │   ├── action.h         # Abstract non-blocking output actions (Click, Down/Up, Scroll)
│   │   └── config.h         # Runtime configuration struct, calibrated defaults, CLI parser
│   ├── features/
│   │   ├── tap_gesture.h    # Pure, non-blocking tap, double-click, and drag state machine API
│   │   └── scroll_wheel.h   # 2D kinetic scroll state machine API with axis-lock deadband
│   └── io/
│       ├── device.h         # /dev/input discovery & optional xinput speed helper
│       └── emitter.h        # Virtual /dev/uinput mouse & wheel action dispatcher
├── src/
│   ├── main.c               # Daemon entry point, signal handling, and epoll event loop
│   ├── core/
│   │   └── config.c         # Default parameters and command-line argument parser
│   ├── features/
│   │   ├── tap_gesture.c    # Anti-drift physical transducer state machine implementation
│   │   └── scroll_wheel.c   # 2D scroll accumulator and cross-axis deadband filter
│   └── io/
│       ├── device.c         # EVIOCGNAME scanning and safe fork/execv xinput helper
│       └── emitter.c        # uinput device lifecycle and non-blocking frame emission
├── tests/
│   ├── test_tap_gesture.c   # Deterministic unit tests for all tap/drag gesture scenarios
│   └── test_scroll_wheel.c  # Deterministic unit tests for 2D kinetic scrolling
├── Makefile
└── alps-stick.service
```

## Architecture

### 1. Anti-Drift Physical Transducer (Tap-to-Click & Drag)

Rather than attempting to guess user intent through delayed confirmation heuristics, the daemon implements a non-blocking **Anti-Drift Physical Transducer**:

1. **Light, Smooth Tap Activation ($Z \ge 22$):**
   * Keeps the contact floor at $Z=22$, enabling gentle, effortless, smooth tapping.
   * **Force-Ratio Discriminator:** Requires $Z \ge \text{motion} \times 1.5$ for any lateral movement $> 6$ counts, distinguishing true downward taps from steering micro-nudges.

2. **Anti-Drift Snap (Zero Target Slip):**
   * When an index finger strikes a flexible rubber dome, contact shear physically deflects the pointer by a few pixels ($dx, dy$).
   * Upon lift-off, the daemon emits an equal and opposite relative motion pulse ($-dx, -dy$) via `uinput` in the **exact same `SYN_REPORT` frame** as the click.
   * The cursor snaps right back to the pre-tap coordinate before the click lands, completely eliminating cursor slip.

3. **100% Non-Blocking Reactive Engine (Zero-`usleep`):**
   * Minimum click hold intervals (`15ms`) are evaluated strictly via `epoll_wait` timer deadlines (`ALPS_TAP_STATE_TAP1_HOLD` and `ALPS_TAP_STATE_TAP2_HOLD`).
   * The single-threaded event loop never sleeps or blocks, guaranteeing instantaneous response to steering changes.

4. **Zero-Latency Rebound & Drag Lock (Tap 2):**
   * Sensitive rebound threshold ($Z \ge 15$) captures natural second strikes without false triggers, emitting `BTN_LEFT 1` **immediately on touchdown**.
   * **Double-Click:** Quick lift-off without motion releases immediately with anti-drift snap.
   * **Tap-and-Drag:** Steering ($\ge 6$ counts) locks drag from millisecond zero. Tilting the stick sideways drops $Z \to 0$, which is sustained continuously by relative motion with a 140ms inactivity grace.

### 2. High-Resolution 2D Kinetic Scrolling (`features/scroll_wheel`)

Holding the physical trackstick middle button (`BTN_MIDDLE`) and deflecting the stick engages continuous 2D kinetic scrolling:

1. **Continuous Sub-Pixel Wheel (`REL_WHEEL_HI_RES` / `REL_HWHEEL_HI_RES`):**
   * Emits high-resolution scroll deltas (120 units per standard detent notch) for butter-smooth 60Hz/120Hz scrolling in modern browsers and desktop compositors.
   * Maintains sub-pixel remainder accumulators, emitting legacy detent clicks (`REL_WHEEL` / `REL_HWHEEL`) whenever 120 units accumulate for backwards compatibility.

2. **Primary Axis-Lock Deadband Filter:**
   * Natural finger anatomy pushes at slight angles during vertical reading.
   * When vertical scrolling begins, horizontal deflection is filtered through a 4-count deadband to eliminate annoying lateral page jitter. Steering firmly sideways breaks the deadband to permit fluid 2D diagonal panning.

3. **Counter-Drift Cursor Lock:**
   * Emits counteracting relative motion ($-dx, -dy$) in the same frame as scroll wheel packets, anchoring the mouse pointer rock-solid in place while the viewport scrolls.

---

## Installation & Testing

### Prerequisites

```bash
sudo apt install build-essential gcc
```

### Build, Test, & Sanitize

Deterministic test suites run without root privileges or hardware access:

```bash
make
make test

# Run test suites under AddressSanitizer and UndefinedBehaviorSanitizer
make sanitize
```

### Install Systemd Service

```bash
sudo make install
```

This compiles `bin/alps-stick`, installs the binary to `/usr/local/bin/`, sets up the systemd service at `/etc/systemd/system/alps-stick.service`, and enables it to run on boot.

### Runtime CLI Options

All physical thresholds and wheel behaviors can be tuned at runtime without recompiling:

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

2D Kinetic Scroll Options:
      --no-scroll              Disable middle-button trackstick scrolling
      --scroll-speed <mult>    Hi-res units per count [1..120] (default: 15)
      --no-axis-lock           Disable cross-axis deadband (allow free 2D diagonal)
      --no-scroll-counter-drift Allow cursor to move freely during scroll
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
