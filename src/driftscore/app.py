"""
The terminal app: connect to AC, read telemetry, print the live signals.

Run it (after `pip install -e .`, on the Windows machine running AC):

    python -m driftscore.app
"""

import math
import time

from driftscore.signals import slip_angle_deg
from driftscore.telemetry import open_physics, read_physics


def main():
    print("Connecting to Assetto Corsa shared memory...")
    try:
        mm = open_physics()
    except Exception as exc:
        print(f"  Could not connect: {exc}")
        print("  Is AC running on THIS machine with a session loaded?")
        return

    print("Connected. Get on track and drive -- Ctrl+C to stop.\n")

    SPEED_FLOOR = 5.0  # km/h. Below this, slip angle is just noise, so zero it.

    try:
        while True:
            phys = read_physics(mm)
            speed = phys.speedKmh
            slip = 0.0 if speed < SPEED_FLOOR else slip_angle_deg(phys.velocity, phys.heading)

            # TEMP DIAGNOSTIC: print the two raw signals slip is computed
            # from, so we can work out AC's heading convention on real data.
            # Revert this commit once the formula is settled.
            travel_deg = math.degrees(math.atan2(phys.velocity[0], phys.velocity[2]))
            heading_deg = math.degrees(phys.heading)

            # \r keeps rewriting one line so it reads like a live gauge.
            print(
                f"speed {speed:6.1f} | slip {slip:+7.1f} | "
                f"travel {travel_deg:+7.1f} | heading {heading_deg:+7.1f}",
                end="\r",
                flush=True,
            )
            time.sleep(1 / 60)   # ~60 Hz, roughly AC's physics update rate
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        mm.close()


if __name__ == "__main__":
    main()
