"""
Pure signal math: numbers in, numbers out.

Nothing in this module touches shared memory, the OS, or AC itself, so it
runs -- and can be tested -- on any machine, even though the telemetry
source is Windows-only. Drift scoring will live here too, once designed.
"""

import math


def slip_angle_deg(velocity, heading) -> float:
    """
    The core drift signal: the angle between where the car is POINTING and
    where it's actually MOVING.

        ~0 deg  -> gripping / going where it points (normal driving)
        large   -> the back has stepped out, car is sliding sideways (drift)

    We work in the horizontal plane. In AC, x/z are horizontal and y is up,
    so we ignore velocity[1] (the vertical component).
    """
    vx, _vy, vz = velocity[0], velocity[1], velocity[2]

    travel_dir = math.atan2(vx, vz)     # direction of motion, world space
    angle = travel_dir - heading + math.pi   # +pi corrects AC's heading reference (180deg offset)
    # Wrap into a clean [-180, 180] range.
    angle = math.degrees(angle)
    angle = (angle + 180.0) % 360.0 - 180.0
    return angle
