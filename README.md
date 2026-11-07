# ALPS DualPoint Trackstick Gesture Daemon

A lightweight, native Linux userspace daemon written in pure C that restores **tap-to-click, double-click, tap-and-drag gestures**, and **high-resolution 2D kinetic scrolling** to ALPS DualPoint pointing sticks.

It operates by intercepting raw Linux kernel `evdev` pressure and button streams from `/dev/input/event*`, evaluating contact dynamics through deterministic finite state machines, and synthesizing clean mouse and wheel events via `/dev/uinput` with zero CPU busy-polling.

---

> [!IMPORTANT]
> ### System Compatibility & Target Environment
> - **Primary Target**: **Linux on Wayland** using modern compositors with `libinput` (specifically developed and verified on **KDE Plasma 5/6** with `kwin_wayland`).
> - **Hardware Requirement**: Any laptop equipped with an ALPS DualPoint pointing stick (reporting downward force via kernel `evdev` as `ABS_PRESSURE`, typically discovered as `AlpsPS/2 ALPS DualPoint Stick`).
> - **X11 / Other Desktops Notice**: Tap-to-click, double-click, and drag gestures work universally across all Linux environments because they rely solely on standard kernel `evdev` and `uinput`. However, middle-button scrolling relies on the compositor's native pointer locking behavior (`scrollOnButtonDown`). If running under standalone X11 without a modern desktop environment, consult the [X11 & Desktop Compatibility Guide](#display-server--desktop-compatibility) below.

---

## Quick Start (3 Steps)

### 1. Build and Install

```bash
# Install standard build dependencies (Debian/Ubuntu)
sudo apt install build-essential gcc

# Compile and run test suite
make
make test

# Install binary to /usr/local/bin and enable systemd service on boot
sudo make install
```

### 2. Calibrate to Your Natural Pressure Style

Every trackstick sensor and finger has different physical resistance. Launch the interactive Control Center to run the guided 4-phase calibration wizard:

```bash
sudo alps-stick -i
```
Select `1` to run the calibration wizard. It guides you through hands-off resting baseline, single taps, double-clicks, and circle steering, then automatically saves tailored parameters to `/etc/alps-stick.conf` and offers to restart the service.

### 3. Verify Service Status

```bash
systemctl status alps-stick.service
```

Tap the rubber nub with your index finger—it now behaves just like a physical left button!

---

## Why This Daemon Exists

Under modern Linux desktop environments, pointing sticks are treated strictly as relative motion pointers (`REL_X`, `REL_Y`). While the kernel's `alps.c` driver reads downward strain-gauge force from the nub and emits it as `ABS_PRESSURE`, modern desktop input stacks (`libinput`) ignore pressure on pointing sticks because tap gestures are traditionally defined only for touchpads.

This daemon bridges that gap completely in userspace:
1. It connects directly to the pointing stick's `/dev/input/event*` node.
2. It evaluates pressure dynamics, contact dwell time, velocity memory, and lateral shear.
3. It emits virtual mouse button down/up events and smooth wheel scrolls via `/dev/uinput`.
4. It does **not** alter native pointer motion during navigation, fully preserving your desktop environment's pointer acceleration, velocity curves, and sensitivity settings.

---

## Core Capabilities & Technical Details

### 1. Feather-Light Tap-to-Click & Drag Lock

Rather than introducing artificial input lag with delayed confirmation heuristics, the daemon implements an instant, non-blocking transducer:

* **Lightweight Activation Floor ($Z \ge 22$):** Enables gentle, effortless tapping without finger fatigue.
* **Force-Ratio Discriminator:** Requires $Z \ge \text{motion} \times 1.5$ for any lateral movement $> 6$ counts, distinguishing true downward taps from steering micro-nudges.
* **Pre-Motion Velocity Memory:** Tracks motion over a decaying 150ms window. If you were steering rapidly across the screen right before pressing the nub, the tap is rejected to prevent accidental misclicks while navigating.
* **Anti-Drift Snap (Zero Target Slip):** When striking a rubber dome cap, contact shear physically deflects the cursor by 1–3 pixels. Upon lift-off, the daemon emits an equal and opposite relative motion pulse ($-dx, -dy$) in the **exact same `SYN_REPORT` frame** as the click. The cursor snaps back to where your finger originally touched, preventing missed clicks on tiny UI targets.
* **Zero-Latency Rebound (Tap 2):** Sensitive rebound threshold ($Z \ge 15$) captures natural second strikes, asserting `BTN_LEFT 1` **immediately on touchdown**.
* **Tap-and-Drag:** Steering ($\ge 6$ counts) locks dragging from millisecond zero. Tilting the stick sideways drops pressure ($Z \to 0$), which is sustained continuously by motion with a 140ms inactivity grace period.

### 2. High-Resolution 2D Kinetic Scrolling

Holding the physical trackstick middle button (`BTN_MIDDLE`) and nudging the stick engages smooth continuous 2D scrolling:

* **Continuous Sub-Pixel Wheel (`REL_WHEEL_HI_RES` / `REL_HWHEEL_HI_RES`):** Emits 120 units per standard detent notch, delivering butter-smooth 60Hz/120Hz kinetic scrolling in modern web browsers, documents, and IDEs.
* **Dual-Range Progressive Ballistics:** Micro-stepping precision ($|v| \le 2$) for single-line reading, transitioning smoothly into a quadratic power curve ($|v| \ge 3$) for rapid gliding down long files.
* **Continuous $C^1$ Hermite Angle Blending:** An 8-bit fixed-point cubic Hermite smoothstep filter ($S(t) = 3t^2 - 2t^3$) deadbands lateral tremor while reading vertically, eliminating accidental browser back/forward page navigation. Steering deliberately sideways smoothly opens full 2D diagonal panning without jarring steps.
* **Clean Event Isolation:** The scroll engine strictly emits wheel events only. It does not inject artificial relative cursor displacements, allowing the display compositor to anchor the pointer rock-solid in place.

---

## Display Server & Desktop Compatibility

### Wayland (KDE Plasma, GNOME, Sway, Hyprland)
* **Status: Fully Supported & Verified.**
* Modern Wayland compositors backed by `libinput` natively recognize pointing sticks (`ID_INPUT_POINTINGSTICK=1`) and automatically enable on-button-down scrolling (`scrollOnButtonDown = true` with `scrollButton = 274 / BTN_MIDDLE`).
* When middle-click scrolling is engaged, the compositor natively locks the pointer on screen while accepting the daemon's high-resolution `REL_WHEEL_HI_RES` stream.

### X11 & Standalone Window Managers (i3, bspwm, Openbox)
* **Status: Supported with Standard Configuration.**
* Tap gestures, double-clicks, and dragging work identically out of the box.
* For middle-button scrolling, ensure that `libinput` manages your trackstick with button scrolling enabled so the X server suppresses cursor movement during scroll. You can verify this using `xinput`:
  ```bash
  # Check if libinput button scrolling is enabled on your stick
  xinput list-props "AlpsPS/2 ALPS DualPoint Stick" | grep -i "scroll method"
  
  # Enable on-button-down scroll method if disabled (0, 0, 1 = Button Scrolling)
  xinput set-prop "AlpsPS/2 ALPS DualPoint Stick" "libinput Scroll Method Enabled" 0 0 1
  ```
* If you prefer pure native desktop scrolling without the daemon's 2D kinetic ballistics engine, you can simply set `enabled = false` under `[scroll]` in `/etc/alps-stick.conf`.

---

## Interactive Tooling

### Control Center (`-i, --interactive`)
Run the unified C terminal dashboard to manage all daemon features in one place:
```bash
sudo alps-stick -i
```
```text
====================================================
   ALPS Trackstick Daemon - Control Center (v0.9.1)
====================================================
  1) Run Guided Calibration Wizard (4-Phase)
  2) Live Diagnostic Sensor HUD (Z-force & shear telemetry)
  3) Reset Configuration to Factory Defaults
  4) Restart Daemon Service (systemctl restart alps-stick)
  5) View Current Configuration (/etc/alps-stick.conf)
  0) Exit
====================================================
```

### Guided Physical Calibration Wizard (`-C, --calibrate`)
Profiles your individual strain-gauge sensor across 4 paced phases:
1. **Quiescent Resting Baseline**: Samples hands-off noise floor for 2.0 seconds (press Enter when ready).
2. **Natural Single Taps**: Measures peak contact force, dwell duration, and shear slip across 5 taps (with retry option).
3. **Double-Click Cadence**: Measures second-strike rebound force loss and cadence gap across 3 double-clicks.
4. **Steering Shear Profile**: Explains circle navigation, waits for finger placement, counts down `3... 2... 1... GO!`, and measures navigation shear limits.
Automatically computes optimal thresholds and saves them to `/etc/alps-stick.conf`.

### Factory Reset (`-R, --reset-config`)
Restores clean, documented factory default configuration:
```bash
sudo alps-stick --reset-config
```

---

## Configuration Reference (`/etc/alps-stick.conf`)

Physical parameters can be tuned in `/etc/alps-stick.conf` or overridden at runtime via CLI arguments:

```ini
[general]
# Enable live gesture telemetry to stdout / journal
verbose = false

# Optional libinput pointer speed override [-1.0 to 1.0].
# Unset by default to preserve native desktop environment (KDE/GNOME) settings.
# pointer_speed = 0.0

[tap]
# Initial tap activation pressure threshold [1..255] (default: 22)
tap1_pressure = 22

# Sensitive rebound threshold [1..255] for rapid double clicks (default: 15)
tap2_pressure = 15

# Mechanical unload baseline [0..255] where finger contact is released (default: 10)
release_pressure = 10

# Maximum contact duration in milliseconds for an impulse tap (default: 180 ms)
tap_duration_ms = 180

# Maximum permitted lateral shear motion counts during a tap (default: 16)
tap_motion = 16

# Primed double-tap recognition window in milliseconds (default: 260 ms)
double_tap_window_ms = 260

# Lateral motion threshold on Tap 2 that locks into active dragging (default: 6)
drag_start_motion = 6

# Inactivity grace window in milliseconds before dropping an active drag (default: 140 ms)
drag_release_grace_ms = 140

# Reverse contact-shear snap on tap release to counteract finger bounce (default: true)
anti_drift = true

[scroll]
# Enable 2D kinetic scrolling using physical middle-button hold (default: true)
enabled = true

# Base high-resolution scroll units per motion count [1..60] (default: 12)
hires_multiplier = 12

# Enable dual-range progressive quadratic ballistics acceleration (default: true)
progressive = true

# Acceleration power factor for progressive curve [0..20] (default: 6)
accel_factor = 6

# Enable continuous C^1 Hermite angle blending and vector cone deadband (default: true)
axis_lock = true
```

---

## Command-Line Usage

```text
Usage: alps-stick [OPTIONS]

General Options:
  -c, --config <path>          Path to configuration file (default: /etc/alps-stick.conf)
  -i, --interactive            Open unified interactive terminal Control Center
  -C, --calibrate              Launch guided 4-phase physical calibration wizard
  -R, --reset-config           Reset configuration file to factory defaults and exit
  -f, --force                  Bypass confirmation prompts for destructive operations
  -v, --verbose                Enable live gesture telemetry and state logging
  -s, --speed <float>          Override libinput pointer speed [-1.0 to 1.0]
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
      --scroll-speed <mult>    Hi-res units per count [1..60] (default: 12)
      --linear-scroll          Disable progressive ballistics (force linear scaling)
      --scroll-accel <power>   Progressive acceleration power factor [0..20] (default: 6)
      --no-axis-lock           Disable cross-axis deadband (allow raw 2D diagonal)
```

---

## Service Management

```bash
# Check service status
systemctl status alps-stick.service

# View live daemon logs
journalctl -u alps-stick.service -f

# Restart service after editing /etc/alps-stick.conf
sudo systemctl restart alps-stick.service
```

---

## Uninstallation

To completely remove the daemon and service:
```bash
sudo make uninstall
```

---

## License

MIT
