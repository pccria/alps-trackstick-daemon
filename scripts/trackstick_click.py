#!/usr/bin/env python3
"""
ALPS DualPoint Stick Gesture & Tap Daemon (Python)
Matches the hardened native C daemon behavior:
  - Immediate Tap-2 Button Lock (zero-slip double click & drag)
  - Motion-Coupled Drag Retention (prevents mid-flight drops during steering)
  - Safe resource lifecycle (no FD leaks, clean disconnect handling, safe signal trapping)
  - Automatic pointer speed boost via /usr/bin/xinput
"""

import argparse
import errno
import selectors
import signal
import subprocess
import sys
import time
import evdev
from evdev import UInput, ecodes

MOTION_CAP = 10000


class AlpsTrackstick:
    STATE_IDLE = "IDLE"
    STATE_TAP1_DOWN = "TAP1_DOWN"
    STATE_AWAITING_TAP2 = "AWAITING_TAP2"
    STATE_TAP2_DOWN = "TAP2_DOWN"

    def __init__(self, tap_pressure=24, release_pressure=10,
                 max_tap_time=0.18, max_tap1_motion=6,
                 double_tap_window=0.35, drag_release_grace=0.12,
                 speed=0.8, quiet=False):
        self.tap_pressure = tap_pressure
        self.release_pressure = release_pressure
        self.max_tap_time = max_tap_time
        self.max_tap1_motion = max_tap1_motion
        self.double_tap_window = double_tap_window
        self.drag_release_grace = drag_release_grace
        self.speed = speed
        self.quiet = quiet

        self.running = True
        self.state = self.STATE_IDLE

        self.press_time = 0.0
        self.tap1_release_time = 0.0
        self.last_active_time = 0.0
        self.max_pressure = 0
        self.tap1_motion = 0
        self.drag_motion = 0
        self.is_drag_locked = False

        self.device = None
        self.uinput = None
        self.selector = None

        try:
            self.device = self._find_device()
            self.uinput = self._create_uinput()
            self.selector = selectors.DefaultSelector()
            self.selector.register(self.device.fd, selectors.EVENT_READ)
            self._apply_pointer_speed()
        except Exception:
            self.cleanup()
            raise

    def _find_device(self):
        for path in evdev.list_devices():
            dev = None
            try:
                dev = evdev.InputDevice(path)
                if "DualPoint Stick" in dev.name:
                    print(f"Connected to: {dev.name} ({dev.path})")
                    print("Gestures: Tap (Click) | Double-Tap (Double Click) | Double-Tap-Hold (Drag)")
                    print("-------------------------------------------------------------------------")
                    return dev
                dev.close()
            except OSError:
                if dev is not None:
                    try:
                        dev.close()
                    except OSError:
                        pass
        print("Error: ALPS DualPoint Stick not found!", file=sys.stderr)
        sys.exit(1)

    def _create_uinput(self):
        cap = {ecodes.EV_KEY: [ecodes.BTN_LEFT]}
        return UInput(cap, name="alps-tap-emitter")

    def _apply_pointer_speed(self):
        if self.speed is None or not (-1.0 <= self.speed <= 1.0):
            return
        try:
            subprocess.run(
                ["/usr/bin/xinput", "set-prop", self.device.name, "libinput Accel Speed", f"{self.speed:.2f}"],
                check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            subprocess.run(
                ["/usr/bin/xinput", "set-prop", self.device.name, "libinput Accel Profile Enabled", "1", "0"],
                check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except OSError:
            pass

    def _emit_down(self):
        self.uinput.write(ecodes.EV_KEY, ecodes.BTN_LEFT, 1)
        self.uinput.syn()

    def _emit_up(self):
        self.uinput.write(ecodes.EV_KEY, ecodes.BTN_LEFT, 0)
        self.uinput.syn()

    def _emit_quick_click(self):
        self._emit_down()
        time.sleep(0.015)
        self._emit_up()

    def _log(self, msg):
        if not self.quiet:
            print(msg, flush=True)

    def run(self):
        while self.running:
            now = time.monotonic()
            timeout = None

            if self.state == self.STATE_AWAITING_TAP2:
                remaining = self.double_tap_window - (now - self.tap1_release_time)
                if remaining <= 0:
                    self.state = self.STATE_IDLE
                else:
                    timeout = remaining

            elif self.state == self.STATE_TAP2_DOWN and self.is_drag_locked:
                inactive = now - self.last_active_time
                if inactive >= self.drag_release_grace:
                    self._emit_up()
                    self.state = self.STATE_IDLE
                    self.is_drag_locked = False
                    self._log("  [DRAG RELEASED: Completed]")
                else:
                    timeout = self.drag_release_grace - inactive

            try:
                events = self.selector.select(timeout=timeout)
            except InterruptedError:
                continue

            if not events:
                now = time.monotonic()
                if self.state == self.STATE_AWAITING_TAP2:
                    self.state = self.STATE_IDLE
                elif self.state == self.STATE_TAP2_DOWN and self.is_drag_locked:
                    if (now - self.last_active_time) >= self.drag_release_grace:
                        self._emit_up()
                        self.state = self.STATE_IDLE
                        self.is_drag_locked = False
                        self._log("  [DRAG RELEASED: Completed]")
                continue

            for key, mask in events:
                try:
                    for event in self.device.read():
                        self._handle_event(event)
                except BlockingIOError:
                    pass
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EINTR):
                        continue
                    print(f"Input device disconnected ({e}). Exiting cleanly.", file=sys.stderr)
                    self.running = False
                    break

    def _handle_event(self, event):
        now = time.monotonic()

        # 1. Track Lateral Motion (with saturation ceiling)
        if event.type == ecodes.EV_REL:
            mag = min(abs(event.value), MOTION_CAP)
            if self.state == self.STATE_TAP1_DOWN:
                self.tap1_motion = min(self.tap1_motion + mag, MOTION_CAP)
            elif self.state == self.STATE_TAP2_DOWN:
                self.drag_motion = min(self.drag_motion + mag, MOTION_CAP)
                self.last_active_time = now
                if self.drag_motion >= 6:
                    self.is_drag_locked = True

        # 2. Track Downward Pressure
        elif event.type == ecodes.EV_ABS and event.code == ecodes.ABS_PRESSURE:
            val = event.value

            # --- Touch Down ---
            if val >= self.tap_pressure:
                if self.state == self.STATE_IDLE:
                    self.state = self.STATE_TAP1_DOWN
                    self.press_time = now
                    self.max_pressure = val
                    self.tap1_motion = 0
                elif self.state == self.STATE_TAP1_DOWN:
                    if val > self.max_pressure:
                        self.max_pressure = val
                elif self.state == self.STATE_AWAITING_TAP2:
                    # Immediate lock on Tap 2 touchdown!
                    self.state = self.STATE_TAP2_DOWN
                    self.press_time = now
                    self.last_active_time = now
                    self.drag_motion = 0
                    self.is_drag_locked = False
                    self._emit_down()
                    self._log("  [TAP 2 TOUCHDOWN: Button Locked DOWN immediately]")
                elif self.state == self.STATE_TAP2_DOWN:
                    self.last_active_time = now

            # --- Lift Off / Pressure Dip ---
            elif val <= self.release_pressure:
                if self.state == self.STATE_TAP1_DOWN:
                    duration = now - self.press_time
                    if duration <= self.max_tap_time and self.tap1_motion <= self.max_tap1_motion:
                        self._emit_quick_click()
                        self.tap1_release_time = now
                        self.state = self.STATE_AWAITING_TAP2
                        self._log(f"-> CLICK (peak={self.max_pressure}, time={int(duration*1000)}ms, motion={self.tap1_motion})")
                    else:
                        self.state = self.STATE_IDLE

                elif self.state == self.STATE_TAP2_DOWN:
                    hold_time = now - self.press_time
                    if not self.is_drag_locked and hold_time <= self.max_tap_time and self.drag_motion < 6:
                        self._emit_up()
                        self.state = self.STATE_IDLE
                        self._log("-> DOUBLE CLICK Completed!")
                    else:
                        self.is_drag_locked = True

    def cleanup(self):
        try:
            if self.uinput is not None and self.state == self.STATE_TAP2_DOWN:
                self._emit_up()
        except Exception:
            pass
        if self.selector is not None:
            try:
                self.selector.close()
            except Exception:
                pass
            self.selector = None
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
            self.device = None
        if self.uinput is not None:
            try:
                self.uinput.close()
            except Exception:
                pass
            self.uinput = None


def main():
    parser = argparse.ArgumentParser(description="ALPS Trackstick Gesture Daemon (Python)")
    parser.add_argument("--speed", type=float, default=0.8,
                        help="Pointer acceleration speed from -1.0 to 1.0 (default: 0.8)")
    parser.add_argument("--tap-pressure", type=int, default=24, help="Tap pressure threshold (default: 24)")
    parser.add_argument("-q", "--quiet", action="store_true", help="Suppress terminal output")
    args = parser.parse_args()

    daemon = AlpsTrackstick(
        tap_pressure=args.tap_pressure,
        speed=args.speed,
        quiet=args.quiet
    )

    def handle_signal(sig, frame):
        daemon.running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        daemon.run()
    finally:
        daemon.cleanup()


if __name__ == "__main__":
    main()
