#!/usr/bin/env python3
"""
ALPS Trackstick Gesture Telemetry & Data Collector
Captures raw event streams from the ALPS DualPoint Stick to analyze:
- Peak pressure (Z) on Tap 1 vs Tap 2
- Lateral displacement (X, Y motion counts) during taps vs drags
- Inter-tap time intervals and release baselines
"""

import sys
import time
import evdev
from evdev import ecodes

def find_alps_device():
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for d in devices:
        if "DualPoint Stick" in d.name:
            return d
    return None

def main():
    dev = find_alps_device()
    if not dev:
        print("Error: ALPS DualPoint Stick not found.")
        print("Ensure you have read permissions on /dev/input/event* (e.g., in 'input' group).")
        sys.exit(1)

    print(f"Monitoring: {dev.name} ({dev.path})")
    print("Commands:")
    print("  Perform: 1) Single Clicks  2) Double Clicks  3) Tap-and-Drags  4) Steering")
    print("Press Ctrl+C to exit.\n")
    print(f"{'TIMESTAMP':<14} | {'TYPE':<8} | {'AXIS':<12} | {'VALUE':<6} | {'DESCRIPTION'}")
    print("-" * 65)

    last_time = time.monotonic()
    pressure = 0
    motion_x = 0
    motion_y = 0

    try:
        for event in dev.read_loop():
            now = time.monotonic()
            dt = (now - last_time) * 1000.0
            last_time = now

            if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_PRESSURE:
                pressure = event.value
                desc = "Touchdown" if pressure >= 20 else ("Lift-off" if pressure <= 10 else "Pressure")
                print(f"{now:14.4f} | ABS      | PRESSURE     | {pressure:<6} | {desc} (dt={dt:4.1f}ms)")
            elif event.type == ecodes.EV_REL:
                axis = "REL_X" if event.code == ecodes.REL_X else "REL_Y"
                print(f"{now:14.4f} | REL      | {axis:<12} | {event.value:<6} | Motion (dt={dt:4.1f}ms, Z={pressure})")

    except KeyboardInterrupt:
        print("\nTelemetry session ended.")

if __name__ == "__main__":
    main()
