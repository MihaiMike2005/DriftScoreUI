"""
Tests for the shared-memory layouts in driftscore.telemetry.

These run anywhere -- no Assetto Corsa, no Windows, no shared memory. The
offsets come from AC's documented struct layout; the live check on the AC
PC confirms them. If it doesn't, fix the struct and these numbers together.
"""

import ctypes
import struct

import pytest

from driftscore.telemetry import (
    READ_RETRIES,
    ACGraphicsPrefix,
    ACPhysics,
    ACStaticPrefix,
    read_page,
    wchars_to_str,
)


@pytest.mark.parametrize(
    "layout, field, offset",
    [
        (ACPhysics, "speedKmh", 28),             # app.py reads speedKmh, velocity and heading
        (ACPhysics, "velocity", 32),
        (ACPhysics, "heading", 208),             # unchanged by the extension: a regression guard
        (ACPhysics, "localAngularVel", 296),
        (ACGraphicsPrefix, "status", 4),
        (ACStaticPrefix, "carModel", 68),
        (ACStaticPrefix, "track", 134),
    ],
)
def test_field_offsets_match_ac_layout(layout, field, offset):
    assert getattr(layout, field).offset == offset


@pytest.mark.parametrize(
    "layout, size",
    [(ACPhysics, 308), (ACGraphicsPrefix, 12), (ACStaticPrefix, 200)],
)
def test_struct_sizes_match_ac_layout(layout, size):
    # Catches a wrong last field, which no offset check can see.
    assert ctypes.sizeof(layout) == size


def test_local_angular_vel_decodes_from_byte_300():
    # Offsets on paper aren't enough: bytes AC writes at 300 must come back
    # as localAngularVel[1] when we actually decode a buffer.
    buf = bytearray(ctypes.sizeof(ACPhysics))
    struct.pack_into("<f", buf, 300, -1.25)
    phys = ACPhysics.from_buffer_copy(buf)
    assert phys.localAngularVel[1] == -1.25


def test_read_page_round_trip():
    # A stable page passes the packetId re-check on the first copy, and the
    # values come back from the offsets AC writes them at.
    buf = bytearray(ctypes.sizeof(ACPhysics))
    struct.pack_into("<i", buf, 0, 4242)
    struct.pack_into("<f", buf, 28, 87.5)
    phys = read_page(buf, ACPhysics)
    assert phys.packetId == 4242
    assert phys.speedKmh == 87.5


class _PageWrittenDuringCopy(bytearray):
    # Stands in for AC writing a new packet while read_page copies. The copy
    # goes through the buffer protocol and sees the old bytes; the packetId
    # re-check is a slice, so it lands after the write.
    def __init__(self, size, writes):
        super().__init__(size)
        self.writes = writes   # how many re-checks find a new packet

    def __getitem__(self, key):
        if self.writes:
            self.writes -= 1
            packet_id = struct.unpack_from("<i", self, 0)[0] + 1
            struct.pack_into("<i", self, 0, packet_id)
            struct.pack_into("<f", self, 28, float(packet_id))
        return super().__getitem__(key)


def test_read_page_recopies_when_packet_id_moves():
    # The first copy is from packet 0, but packet 1 landed meanwhile: return
    # packet 1, speed included, not the mix.
    page = _PageWrittenDuringCopy(ctypes.sizeof(ACPhysics), writes=1)
    phys = read_page(page, ACPhysics)
    assert phys.packetId == 1
    assert phys.speedKmh == 1.0


def test_read_page_gives_up_after_read_retries():
    # A page that changes on every re-check must not hang the poll loop.
    page = _PageWrittenDuringCopy(ctypes.sizeof(ACPhysics), writes=100)
    phys = read_page(page, ACPhysics)
    assert phys.packetId == READ_RETRIES


def test_read_page_without_packet_id():
    # The static page has no packetId to re-check; it's a plain copy.
    buf = bytearray(ctypes.sizeof(ACStaticPrefix))
    raw = "ks_toyota_ae86".encode("utf-16-le")
    buf[68:68 + len(raw)] = raw
    static = read_page(buf, ACStaticPrefix)
    assert wchars_to_str(static.carModel) == "ks_toyota_ae86"


def test_wchars_to_str_stops_at_first_nul():
    # Whatever sits after the terminator (e.g. leftovers from a longer name)
    # must not leak into the string.
    arr = (ctypes.c_uint16 * 33)(*map(ord, "ks_toyota_ae86\0junk"))
    assert wchars_to_str(arr) == "ks_toyota_ae86"


def test_wchars_to_str_full_buffer_without_nul():
    # A name that fills all 33 units has no terminator; keep every character.
    name = "abcdefghijklmnopqrstuvwxyzABCDEFG"
    arr = (ctypes.c_uint16 * 33)(*map(ord, name))
    assert wchars_to_str(arr) == name


def test_wchars_to_str_survives_a_lone_surrogate():
    # 0xD800 isn't valid UTF-16 on its own -- the kind of thing a wrong
    # layout hands us. It must decode to the replacement character, not raise.
    arr = (ctypes.c_uint16 * 33)(0xD800)
    assert wchars_to_str(arr) == "\ufffd"
