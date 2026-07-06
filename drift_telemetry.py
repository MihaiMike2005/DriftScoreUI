"""
Assetto Corsa drift rater - Step 1: the telemetry skeleton.

Goal of this step: get live data flowing and print the two signals drift
scoring is built on -- SPEED and SLIP ANGLE. No scoring yet, no UI yet.
Just prove we can read the car and that the numbers move the right way.

>>> IMPORTANT <<<
This talks to Assetto Corsa through Windows shared memory, so it must run on
the SAME Windows machine where AC is running, with a session actually loaded
(car on track, not sitting in a menu). It will NOT connect from Linux/macOS
or from a different PC -- shared memory is local and Windows-only.

Dependencies: none. Everything here is Python standard library.
Run it with:  python drift_telemetry.py
"""

import ctypes
import math
import mmap
import time


# ---------------------------------------------------------------------------
# 1. Describe the part of AC's physics struct we care about.
#
# AC continuously writes a large C struct into shared memory. The fields sit
# at fixed byte offsets, so their ORDER and TYPES must match AC's layout
# exactly -- get one wrong and every field after it reads garbage.
#
# We only need data up to `heading`, so we define just that prefix. We always
# read from offset 0, so the fields AC has *after* heading don't concern us.
# ---------------------------------------------------------------------------
class ACPhysics(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", ctypes.c_int),
        ("gas", ctypes.c_float),
        ("brake", ctypes.c_float),
        ("fuel", ctypes.c_float),
        ("gear", ctypes.c_int),
        ("rpms", ctypes.c_int),
        ("steerAngle", ctypes.c_float),
        ("speedKmh", ctypes.c_float),
        ("velocity", ctypes.c_float * 3),           # world velocity (x, y, z)
        ("accG", ctypes.c_float * 3),
        ("wheelSlip", ctypes.c_float * 4),
        ("wheelLoad", ctypes.c_float * 4),
        ("wheelsPressure", ctypes.c_float * 4),
        ("wheelAngularSpeed", ctypes.c_float * 4),
        ("tyreWear", ctypes.c_float * 4),
        ("tyreDirtyLevel", ctypes.c_float * 4),
        ("tyreCoreTemperature", ctypes.c_float * 4),
        ("camberRAD", ctypes.c_float * 4),
        ("suspensionTravel", ctypes.c_float * 4),
        ("drs", ctypes.c_float),
        ("tc", ctypes.c_float),
        ("heading", ctypes.c_float),                # car yaw in radians
        # AC has many more fields after this; we stop because we don't use them.
    ]


SHARED_MEMORY_NAME = "acpmf_physics"   # the Windows named mapping AC creates


def open_physics():
    """Connect to AC's physics shared memory and return an mmap handle."""
    size = ctypes.sizeof(ACPhysics)
    # fileno=-1 + tagname connects to the existing mapping AC already created.
    return mmap.mmap(-1, size, tagname=SHARED_MEMORY_NAME, access=mmap.ACCESS_READ)


def read_physics(mm) -> ACPhysics:
    """Snapshot the current bytes out of shared memory into a struct."""
    mm.seek(0)
    raw = mm.read(ctypes.sizeof(ACPhysics))
    return ACPhysics.from_buffer_copy(raw)


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
    angle = travel_dir - heading        # vs. where the nose points

    # Wrap into a clean [-180, 180] range.
    angle = math.degrees(angle)
    angle = (angle + 180.0) % 360.0 - 180.0
    return angle


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

            # \r keeps rewriting one line so it reads like a live gauge.
            print(
                f"speed: {speed:6.1f} km/h   |   slip angle: {slip:+7.1f} deg",
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
