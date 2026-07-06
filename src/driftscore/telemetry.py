"""
Reading Assetto Corsa's physics shared memory.

AC continuously writes a C struct into Windows shared memory; this module
knows that struct's layout and how to snapshot it. All Windows-specific
code lives HERE -- the rest of the package works on plain Python values
and runs (and is testable) on any OS.

>>> IMPORTANT <<<
This talks to Assetto Corsa through Windows shared memory, so it must run on
the SAME Windows machine where AC is running, with a session actually loaded
(car on track, not sitting in a menu). It will NOT connect from Linux/macOS
or from a different PC -- shared memory is local and Windows-only.
"""

import ctypes
import mmap


# ---------------------------------------------------------------------------
# Describe the part of AC's physics struct we care about.
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
