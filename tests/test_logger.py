"""
Tests for the pure parts of driftscore.logger.

These run anywhere -- no Assetto Corsa, no Windows, no shared memory. The
poll loop itself stays untested, like app.py.
"""

import argparse
from datetime import datetime
from pathlib import Path

import pytest

from driftscore.logger import (
    COLUMNS,
    main,
    parse_poll_ms,
    physics_row,
    session_paths,
    session_summary,
    should_write,
    status_line,
    status_name,
    track_break,
)
from driftscore.telemetry import AC_LIVE, AC_OFF, AC_PAUSE, AC_REPLAY, ACPhysics


def test_physics_row_columns_and_values():
    # Every column gets a distinct value, exact in float32, so a swapped or
    # dropped field can't pass. The expected dict's keys also pin COLUMNS,
    # the CSV header spinwarn reads.
    phys = ACPhysics(
        packetId=4242,
        speedKmh=87.5,
        velocity=(1.5, -0.25, 23.75),
        heading=-1.125,
        localAngularVel=(0.0625, -0.875, 0.1875),
        accG=(0.375, -0.03125, 1.25),
        steerAngle=-0.5,
        gas=0.75,
        brake=0.125,
        gear=3,
        rpms=6500,
        wheelSlip=(0.5, 0.625, 2.5, 3.5),
    )
    expected = {
        "t": 12.5, "packet_id": 4242,
        "speed_kmh": 87.5, "vel_x": 1.5, "vel_y": -0.25, "vel_z": 23.75, "heading": -1.125,
        "ang_vel_x": 0.0625, "ang_vel_y": -0.875, "ang_vel_z": 0.1875,
        "acc_g_x": 0.375, "acc_g_y": -0.03125, "acc_g_z": 1.25,
        "steer": -0.5, "gas": 0.75, "brake": 0.125, "gear": 3, "rpms": 6500,
        "wheel_slip_fl": 0.5, "wheel_slip_fr": 0.625, "wheel_slip_rl": 2.5, "wheel_slip_rr": 3.5,
    }
    row = physics_row(phys, 12.5)
    assert list(expected) == COLUMNS
    assert row == list(expected.values())


def test_physics_row_trims_float32_noise():
    # 0.3 is stored as 0.30000001192092896 in float32; the CSV should say 0.3.
    phys = ACPhysics(gas=0.3)
    assert phys.gas != 0.3
    row = physics_row(phys, 0.0)
    assert row[COLUMNS.index("gas")] == 0.3


@pytest.mark.parametrize(
    "packet_id, last_packet_id, status, expected",
    [
        (101, 100, AC_LIVE, True),       # new live packet
        (100, 100, AC_LIVE, False),      # duplicate: polled before AC wrote the next one
        (101, 100, AC_PAUSE, False),
        (101, 100, AC_REPLAY, False),
        (7, None, AC_LIVE, True),        # first packet of the session
        (3, 5000, AC_LIVE, True),        # packetId went backwards: still a new packet
    ],
)
def test_should_write(packet_id, last_packet_id, status, expected):
    assert should_write(packet_id, last_packet_id, status) is expected


@pytest.mark.parametrize(
    "wrote, status, rows, left_live, expected",
    [
        (True, AC_LIVE, 5, False, (False, False)),     # ordinary live row
        (False, AC_LIVE, 5, False, (False, False)),    # duplicate packet: a hiccup, not a break
        (False, AC_PAUSE, 5, False, (False, True)),    # paused after rows: next row is a break
        (False, AC_REPLAY, 5, False, (False, True)),
        (False, AC_OFF, 5, False, (False, True)),
        (False, AC_PAUSE, 0, False, (False, False)),   # paused before the first row: nothing to leave out
        (False, AC_LIVE, 5, True, (False, True)),      # resumed, packetId not moved yet: still waiting
        (True, AC_LIVE, 5, True, (True, False)),       # first row after the pause is the break
    ],
)
def test_track_break(wrote, status, rows, left_live, expected):
    assert track_break(wrote, status, rows, left_live) == expected


@pytest.mark.parametrize("text, value", [("0.1", 0.1), ("1", 1.0), ("1000", 1000.0)])
def test_parse_poll_ms(text, value):
    assert parse_poll_ms(text) == value


@pytest.mark.parametrize("text", ["0", "0.001", "0.09", "-1", "nan", "inf", "1000.5", "1e20", "abc"])
def test_parse_poll_ms_refuses(text):
    with pytest.raises(argparse.ArgumentTypeError):
        parse_poll_ms(text)


@pytest.mark.parametrize(
    "text, message",
    [
        ("-1", "--poll-ms: must be from 0.1 to 1000, got -1"),
        ("abc", "--poll-ms: not a number: 'abc'"),
    ],
)
def test_bad_poll_ms_stops_before_connecting(capsys, text, message):
    # argparse exits before main connects to AC, so no files get created.
    with pytest.raises(SystemExit) as exc:
        main(["--kind", "natural", "--poll-ms", text])
    assert exc.value.code == 2
    out, err = capsys.readouterr()
    assert "Connecting" not in out
    assert message in err


@pytest.mark.parametrize(
    "started, stem",
    [
        (datetime(2026, 10, 11, 14, 30, 0), "2026-10-11_143000"),
        (datetime(2026, 1, 2, 3, 4, 5), "2026-01-02_030405"),   # zero-padded
    ],
)
def test_session_paths(started, stem):
    csv_path, json_path = session_paths("data/raw", started)
    assert csv_path == Path("data/raw") / f"{stem}.csv"
    assert json_path == Path("data/raw") / f"{stem}.json"


def test_session_summary():
    # Rows every 10 ms with one 50 ms gap; packetId steps of 1, 2, 3 and 5.
    ts = [0.00, 0.01, 0.02, 0.03, 0.08, 0.09]
    ids = [10, 11, 13, 16, 21, 22]
    summary = session_summary(ts, ids)
    assert summary["rows"] == 6
    assert summary["duration_s"] == pytest.approx(0.09)
    assert summary["mean_rows_per_s"] == pytest.approx(55.6)
    assert summary["packet_id_steps"] == {"1": 2, "2": 1, "3+": 2, "reset": 0}
    assert summary["longest_gap_s"] == pytest.approx(0.05)


def test_session_summary_leaves_out_breaks_but_not_hiccups():
    # A 0.3 s hiccup while live, then a 2 s pause before row 4. The pause
    # is out of every number; the hiccup still shows as the longest gap.
    ts = [0.00, 0.01, 0.02, 0.32, 2.32, 2.33]
    ids = [1, 2, 3, 4, 500, 501]
    summary = session_summary(ts, ids, breaks=[4])
    assert summary["rows"] == 6
    assert summary["duration_s"] == pytest.approx(0.33)
    assert summary["mean_rows_per_s"] == pytest.approx(12.1)
    assert summary["packet_id_steps"] == {"1": 4, "2": 0, "3+": 0, "reset": 0}
    assert summary["longest_gap_s"] == pytest.approx(0.3)
    assert summary["breaks"] == 1


def test_session_summary_counts_reset():
    summary = session_summary([0.0, 0.01], [500, 0])
    assert summary["packet_id_steps"] == {"1": 0, "2": 0, "3+": 0, "reset": 1}


@pytest.mark.parametrize("ts, ids", [([], []), ([1.5], [7])])
def test_session_summary_without_intervals(ts, ids):
    # Ctrl+C before AC went live, or after a single packet: no crash.
    summary = session_summary(ts, ids)
    assert summary["rows"] == len(ts)
    assert summary["duration_s"] == 0.0
    assert summary["mean_rows_per_s"] == 0.0
    assert summary["longest_gap_s"] == 0.0


def test_status_name():
    assert status_name(AC_LIVE) == "LIVE"
    assert status_name(7) == "7"   # out of range, e.g. a misread page


def test_status_line_fits_80_columns():
    # Widest realistic values: hours in, millions of packets, a backward
    # slide at full slip. Wider would wrap a narrow console every update.
    line = status_line(9999.99, 99_999_999, 9_999_999, 999, AC_REPLAY, 999.9, -180.0)
    assert len(line) < 80
