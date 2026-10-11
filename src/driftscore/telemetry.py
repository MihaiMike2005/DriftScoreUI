"""
Reading Assetto Corsa's shared memory.

AC continuously writes C structs into Windows shared memory -- one page each
for physics, graphics and static (car/track) info; this module knows those
layouts and how to snapshot them. All Windows-specific code lives HERE --
the rest of the package works on plain Python values and runs (and is
testable) on any OS.

>>> IMPORTANT <<<
This talks to Assetto Corsa through Windows shared memory, so it must run on
the SAME Windows machine where AC is running, with a session actually loaded
(car on track, not sitting in a menu). It will NOT connect from Linux/macOS
or from a different PC -- shared memory is local and Windows-only.
"""

import ctypes
import mmap
import sys
from enum import IntEnum


# ---------------------------------------------------------------------------
# Describe the part of AC's physics struct we care about.
#
# AC continuously writes a large C struct into shared memory. The fields sit
# at fixed byte offsets, so their ORDER and TYPES must match AC's layout
# exactly -- get one wrong and every field after it reads garbage.
#
# We need data up to `localAngularVel` (yaw rate), so we define just that
# prefix. We always read from offset 0, so the fields AC has *after* it
# don't concern us.
#
# `_layout_ = "ms"` spells out what `_pack_` already implied: Python 3.14
# deprecates `_pack_` without it. Older Pythons ignore the attribute.
# ---------------------------------------------------------------------------
class ACPhysics(ctypes.Structure):
    _layout_ = "ms"
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
        # Mostly unused, but they must be here to put localAngularVel at the
        # right offset.
        ("pitch", ctypes.c_float),
        ("roll", ctypes.c_float),
        ("cgHeight", ctypes.c_float),
        ("carDamage", ctypes.c_float * 5),
        ("numberOfTyresOut", ctypes.c_int),
        ("pitLimiterOn", ctypes.c_int),
        ("abs", ctypes.c_float),
        ("kersCharge", ctypes.c_float),
        ("kersInput", ctypes.c_float),
        ("autoShifterOn", ctypes.c_int),
        ("rideHeight", ctypes.c_float * 2),
        ("turboBoost", ctypes.c_float),
        ("ballast", ctypes.c_float),
        ("airDensity", ctypes.c_float),
        ("airTemp", ctypes.c_float),
        ("roadTemp", ctypes.c_float),
        ("localAngularVel", ctypes.c_float * 3),    # car frame; which axis is yaw is checked on live data
        # AC has many more fields after this; we stop because we don't use them.
    ]


# ---------------------------------------------------------------------------
# The graphics page, cut down to the prefix that tells live driving apart
# from pause and replay. Same trick as ACPhysics.
# ---------------------------------------------------------------------------
class ACGraphicsPrefix(ctypes.Structure):
    _layout_ = "ms"
    _pack_ = 4
    _fields_ = [
        ("packetId", ctypes.c_int),
        ("status", ctypes.c_int),                   # an ACStatus value, see below
        ("session", ctypes.c_int),
    ]


# AC's status enum, as found in ACGraphicsPrefix.status. An IntEnum so it
# still compares equal to the raw int, and ACStatus(status).name prints --
# but ACStatus() raises ValueError outside 0-3, so fall back to the raw int
# when printing a live read.
class ACStatus(IntEnum):
    OFF = 0
    REPLAY = 1
    LIVE = 2
    PAUSE = 3


AC_OFF, AC_REPLAY, AC_LIVE, AC_PAUSE = ACStatus


# ---------------------------------------------------------------------------
# The static page prefix: versions, car and track, fixed for a session.
#
# AC's strings are Windows wchar_t arrays, i.e. 2-byte UTF-16 units. We use
# c_uint16 rather than c_wchar because c_wchar is 4 bytes on macOS/Linux,
# which would make the layout -- and the tests -- depend on the OS.
# ---------------------------------------------------------------------------
class ACStaticPrefix(ctypes.Structure):
    _layout_ = "ms"
    _pack_ = 4
    _fields_ = [
        ("smVersion", ctypes.c_uint16 * 15),
        ("acVersion", ctypes.c_uint16 * 15),
        ("numberOfSessions", ctypes.c_int),
        ("numCars", ctypes.c_int),
        ("carModel", ctypes.c_uint16 * 33),
        ("track", ctypes.c_uint16 * 33),
    ]


def wchars_to_str(arr) -> str:
    """Decode one of AC's fixed-size UTF-16 strings, up to the first NUL."""
    # bytes(arr) is in native byte order -- little-endian on every machine
    # this runs on, same as AC. A wrong layout should show up as odd
    # characters, not crash the caller.
    return bytes(arr).decode("utf-16-le", errors="replace").split("\0", 1)[0]


PHYSICS_MEMORY_NAME = "acpmf_physics"   # the Windows named mappings AC creates
GRAPHICS_MEMORY_NAME = "acpmf_graphics"
STATIC_MEMORY_NAME = "acpmf_static"

FILE_MAP_READ = 0x0004   # Win32 access flag for OpenFileMappingW
READ_RETRIES = 3         # re-copies before read_page accepts a possibly torn snapshot


def open_page(name, struct):
    """Connect to one of AC's shared memory pages and return an mmap handle.

    Raises FileNotFoundError if the page isn't there (AC not running).
    """
    # mmap with fileno=-1 + tagname opens the named mapping if it exists --
    # but if it doesn't, it silently creates a new zero-filled one. That
    # reads like a parked car, and being read-only and prefix-sized it can
    # block AC's own page. So make sure AC's page is really there first.
    if sys.platform == "win32":
        kernel32 = ctypes.windll.kernel32
        # A HANDLE is pointer-sized; ctypes' default int return would truncate it.
        kernel32.OpenFileMappingW.restype = ctypes.c_void_p
        handle = kernel32.OpenFileMappingW(FILE_MAP_READ, False, name)
        if not handle:
            raise FileNotFoundError(f"{name}: AC isn't running or no session is loaded")
        kernel32.CloseHandle(ctypes.c_void_p(handle))
    size = ctypes.sizeof(struct)
    return mmap.mmap(-1, size, tagname=name, access=mmap.ACCESS_READ)


def read_page(mm, struct):
    """Snapshot the current bytes of a page into an instance of `struct`."""
    snap = struct.from_buffer_copy(mm)
    if not hasattr(struct, "packetId"):
        return snap
    # AC writes the page from its own process with no lock, so a copy can
    # land mid-write. If packetId moved while we copied, copy again. This is
    # best-effort: AC doesn't document its write order, so it catches a write
    # whose packetId change lands between the copy and the re-check, not
    # every tear.
    field = struct.packetId
    for _ in range(READ_RETRIES):
        current = int.from_bytes(mm[field.offset:field.offset + field.size], "little", signed=True)
        if current == snap.packetId:
            break
        snap = struct.from_buffer_copy(mm)
    return snap


def open_physics():
    """Connect to AC's physics shared memory and return an mmap handle."""
    return open_page(PHYSICS_MEMORY_NAME, ACPhysics)


def read_physics(mm) -> ACPhysics:
    """Snapshot the current bytes out of shared memory into a struct."""
    return read_page(mm, ACPhysics)
