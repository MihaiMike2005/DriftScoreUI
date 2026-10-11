"""
Tests for the pure parts of driftscore.logcheck, on synthetic data.

These run anywhere -- no Assetto Corsa, no recording. Opening the files
stays untested, like app.py.
"""

import csv
import io
import json
import math
import random
import statistics

import pytest

from driftscore.logcheck import (
    DERIV_SPAN_S,
    G,
    accel_refs,
    best_axis,
    break_rows,
    car_frame,
    derivative,
    find_gaps,
    main,
    pearson,
    read_columns,
    read_meta,
    step_stats,
    unwrap,
    verdict,
    wheel_medians,
)
from driftscore.logger import COLUMNS, session_summary
from driftscore.signals import slip_angle_deg


SEED = 0
TICK_S = 0.003   # AC's physics step, ~333 packets/s


def wrap(angle):
    """Into [-pi, pi), the way AC reports heading."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def recording(gaps):
    """t and packetId: 100 rows a packet apart every TICK_S, then a (seconds, packetId step) gap, and so on."""
    t, ids = [0.0], [1000]
    for gap in list(gaps) + [None]:
        for _ in range(99):
            t.append(t[-1] + TICK_S)
            ids.append(ids[-1] + 1)
        if gap:
            t.append(t[-1] + gap[0])
            ids.append(ids[-1] + gap[1])
    return t, ids


# --- unwrap, derivative, pearson, best_axis ---


def test_unwrap_leaves_no_spike_in_derivative():
    # Turning at a steady 2 rad/s: the wrapped heading jumps 2π at ±π.
    t = [k * 0.004 for k in range(2000)]
    heading = [wrap(2.0 * x) for x in t]
    assert max(abs(d) for d in derivative(t, heading) if d is not None) > 100   # the spikes unwrap removes
    rates = [d for d in derivative(t, unwrap(heading)) if d is not None]
    assert rates == pytest.approx([2.0] * len(rates))


def test_unwrap_across_pi():
    assert unwrap([3.1, -3.1, 3.1]) == pytest.approx([3.1, 2 * math.pi - 3.1, 3.1])


def test_best_axis_finds_yaw_and_its_sign():
    # A sine-wave heading on jittery poll times; ang_vel_y = -d(heading)/dt + noise.
    rng = random.Random(SEED)
    t = [0.0]
    for _ in range(3000):
        t.append(t[-1] + rng.uniform(0.002, 0.006))
    heading = [wrap(4.0 * math.sin(0.8 * x)) for x in t]
    yaw = [-3.2 * math.cos(0.8 * x) + rng.gauss(0, 0.1) for x in t]
    pitch = [rng.gauss(0, 0.1) for _ in t]
    roll = [0.3 * math.sin(3.0 * x) + rng.gauss(0, 0.1) for x in t]
    axis, r, sign = best_axis([pitch, yaw, roll], derivative(t, unwrap(heading)))
    assert (axis, sign) == (1, -1)
    assert r < -0.95


def test_derivative_over_jittery_times():
    rng = random.Random(SEED)
    t = [0.0]
    for _ in range(500):
        t.append(t[-1] + rng.uniform(0.002, 0.006))
    d = derivative(t, [5.0 * x + 1.0 for x in t])
    defined = [v for v in d if v is not None]
    assert defined == pytest.approx([5.0] * len(defined))
    # No full span before the first row or after the last.
    assert d[0] is None and d[-1] is None
    assert all(v is not None for x, v in zip(t, d) if DERIV_SPAN_S <= x <= t[-1] - DERIV_SPAN_S)


def test_derivative_never_spans_a_gap():
    # x = t, but 100 higher after a 0.3 s gap: differencing across it would read ~330.
    t1 = [k * 0.01 for k in range(101)]
    t2 = [1.3 + k * 0.01 for k in range(101)]
    t = t1 + t2
    x = t1 + [v + 100.0 for v in t2]
    d = derivative(t, x)
    defined = [v for v in d if v is not None]
    assert defined == pytest.approx([1.0] * len(defined))
    near_gap = [v for tv, v in zip(t, d) if 1.0 - DERIV_SPAN_S / 2 < tv < 1.3 + DERIV_SPAN_S / 2]
    assert near_gap and all(v is None for v in near_gap)
    assert d[50] is not None and d[150] is not None


def test_derivative_never_spans_a_listed_break():
    # The same jump after a 20 ms break: under GAP_S, so only the JSON shows it.
    t1 = [k * 0.01 for k in range(101)]
    t2 = [1.02 + k * 0.01 for k in range(101)]
    t = t1 + t2
    x = t1 + [v + 100.0 for v in t2]
    assert max(v for v in derivative(t, x) if v is not None) > 100   # unlisted, it's differenced across
    d = derivative(t, x, breaks={101})
    defined = [v for v in d if v is not None]
    assert defined == pytest.approx([1.0] * len(defined))
    assert d[50] is not None and d[150] is not None


@pytest.mark.parametrize(
    "a, b, r",
    [
        ([1, 2, 3, 4], [2, 4, 6, 8], 1.0),
        ([1, 2, 3, 4], [8, 6, 4, 2], -1.0),
        ([1, 2, 3, 4], [1, 3, 2, 4], 0.8),
        ([1, 2, 3, 4], [0.1, 0.1, 0.1, 0.1], 0.0),   # a dead axis: constant, so 0, not a crash
        ([0.1, 0.1, 0.1], [1, 2, 3], 0.0),
        ([], [], 0.0),
    ],
)
def test_pearson(a, b, r):
    assert pearson(a, b) == pytest.approx(r)


def test_best_axis_skips_rows_without_a_reference():
    # The None rows (around a gap) would break the correlation if they were used.
    ref = [1.0, 2.0, None, 3.0, 4.0]
    candidates = [[0.0, 0.0, 0.0, 0.0, 0.0], [-1.0, -2.0, 99.0, -3.0, -4.0]]
    assert best_axis(candidates, ref) == (1, pytest.approx(-1.0), -1)


# --- car-frame acceleration ---


@pytest.mark.parametrize(
    "ax, az, heading, lon, lat",
    [
        (0.0, 3.0, 0.0, 3.0, 0.0),            # straight ahead at heading 0: the nose is +z
        (-3.0, 0.0, math.pi / 2, 3.0, 0.0),   # straight ahead at heading π/2: the nose is -x
        (3.0, 0.0, 0.0, 0.0, 3.0),            # along the side at heading 0
        (0.0, 3.0, math.pi / 2, 0.0, 3.0),    # along the side at heading π/2
    ],
)
def test_car_frame(ax, az, heading, lon, lat):
    assert car_frame(ax, az, heading) == pytest.approx((lon, lat), abs=1e-12)


@pytest.mark.parametrize("heading", [0.0, 0.7, math.pi / 2, -2.0, math.pi])
def test_car_frame_nose_matches_slip_angle(heading):
    # Moving along the nose is zero slip in signals' verified convention,
    # and all longitudinal here.
    nose = (-math.sin(heading), math.cos(heading))
    assert slip_angle_deg((nose[0], 0.0, nose[1]), heading) == pytest.approx(0.0, abs=1e-9)
    assert car_frame(nose[0], nose[1], heading) == pytest.approx((1.0, 0.0), abs=1e-12)


def test_accel_refs_in_g():
    # Pointing at heading π/2 (nose -x) and speeding up at 2 m/s².
    t = [k * 0.004 for k in range(500)]
    lon, lat = accel_refs(t, [-2.0 * x for x in t], [0.0] * len(t), [math.pi / 2] * len(t))
    rows = [i for i, v in enumerate(lon) if v is not None]
    assert rows
    assert [lon[i] for i in rows] == pytest.approx([2.0 / G] * len(rows))
    assert [lat[i] for i in rows] == pytest.approx([0.0] * len(rows), abs=1e-9)


# --- rate and gaps ---


def test_step_stats_leaves_gaps_out():
    # Two 2 s pauses with packetId frozen: in, they would drag the rate down to ~45/s.
    stats = step_stats(*recording([(2.0, 1), (2.0, 1)]))
    assert stats["packet_rate"] == pytest.approx(1 / TICK_S)
    assert stats["steps"] == {"1": 297, "2": 0, "3+": 0, "reset": 0}
    assert stats["skip_share"] == 0.0


def test_step_stats_counts_skipped_packets():
    # A 6 ms poll keeps every other packet: half AC's rate in rows, every step a 2.
    t = [k * 0.006 for k in range(201)]
    ids = [2 * k for k in range(200)] + [0]   # and a reset at the end
    stats = step_stats(t, ids)
    assert stats["packet_rate"] == pytest.approx(1 / 0.003)
    assert stats["rows_per_s"] == pytest.approx(1 / 0.006)
    assert stats["steps"] == {"1": 0, "2": 199, "3+": 0, "reset": 1}
    assert stats["skip_share"] == 1.0


def test_break_rows():
    t, _ = recording([(2.0, 1), (0.023, 3)])
    breaks = [
        {"start_s": t[99], "end_s": t[100], "statuses": ["PAUSE"]},
        {"start_s": t[199], "end_s": t[200], "statuses": ["OFF"]},
    ]
    assert break_rows(t, breaks) == {100, 200}
    assert break_rows(t, []) == set()
    assert break_rows(t, None) == set()   # no JSON, or an older one


def test_step_stats_leaves_out_listed_breaks():
    # A 23 ms break with a packetId step of 3: under GAP_S, so only the
    # JSON shows it. Listed, it's left out, as the logger leaves it out.
    t, ids = recording([(0.023, 3)])
    assert step_stats(t, ids)["steps"] == {"1": 198, "2": 0, "3+": 1, "reset": 0}
    assert step_stats(t, ids, {100})["steps"] == {"1": 198, "2": 0, "3+": 0, "reset": 0}


def test_step_stats_leaves_out_a_logging_gap_the_logger_counts():
    # A 0.15 s logging gap that lost 50 packets: the logger counts its step
    # as a 3+; here it's left out, and reported as a gap instead.
    t, ids = recording([(0.15, 50)])
    assert session_summary(t, ids)["packet_id_steps"] == {"1": 198, "2": 0, "3+": 1, "reset": 0}
    assert step_stats(t, ids)["steps"] == {"1": 198, "2": 0, "3+": 0, "reset": 0}


def test_gaps_by_packet_id_alone():
    # No break intervals (no JSON, or an older one): a 2 s gap where packetId
    # moved 1 is a pause; a 0.2 s gap where it moved ~0.2 x rate lost packets.
    t, ids = recording([(2.0, 1), (0.2, 67)])
    gaps = find_gaps(t, ids, step_stats(t, ids)["packet_rate"])
    assert [(g["label"], g["logging"], g["note"]) for g in gaps] == [
        ("pause", False, ""),
        ("logging gap", True, ""),
    ]
    assert gaps[0]["start_s"] == t[99] and gaps[0]["end_s"] == t[100]
    assert gaps[0]["step"] == 1


def test_listed_replay_is_not_a_logging_gap():
    # The JSON lists a REPLAY here, though packetId kept moving: the JSON
    # wins, and the disagreement gets a note.
    t, ids = recording([(2.0, 667)])
    breaks = [{"start_s": t[99], "end_s": t[100], "statuses": ["REPLAY"]}]
    (gap,) = find_gaps(t, ids, step_stats(t, ids)["packet_rate"], breaks)
    assert (gap["label"], gap["logging"]) == ("REPLAY", False)
    assert "REPLAY" in gap["note"] and "packetId moved 667" in gap["note"]


def test_listed_pause_with_frozen_packet_id_has_no_note():
    t, ids = recording([(2.0, 1)])
    breaks = [{"start_s": t[99], "end_s": t[100], "statuses": ["PAUSE", "REPLAY"]}]
    (gap,) = find_gaps(t, ids, step_stats(t, ids)["packet_rate"], breaks)
    assert (gap["label"], gap["logging"], gap["note"]) == ("PAUSE + REPLAY", False, "")


def test_unlisted_frozen_gap_is_a_stall():
    # The JSON lists breaks, not this one: the logger saw AC live, yet
    # packetId froze. No packets lost, so not a logging gap, but noted.
    t, ids = recording([(2.0, 1)])
    (gap,) = find_gaps(t, ids, step_stats(t, ids)["packet_rate"], [])
    assert (gap["label"], gap["logging"]) == ("pause", False)
    assert "live" in gap["note"]


def test_unlisted_moving_gap_is_a_logging_gap():
    t, ids = recording([(0.2, 67)])
    (gap,) = find_gaps(t, ids, step_stats(t, ids)["packet_rate"], [])
    assert (gap["label"], gap["logging"], gap["note"]) == ("logging gap", True, "")


def test_short_listed_break_is_still_listed():
    # A 23 ms break, under GAP_S: shown so the JSON's count adds up, with
    # no note (a prediction of ~8 packets is too few to judge).
    t, ids = recording([(0.023, 3)])
    rate = step_stats(t, ids)["packet_rate"]
    assert find_gaps(t, ids, rate, []) == []
    breaks = [{"start_s": t[99], "end_s": t[100], "statuses": ["OFF"]}]
    (gap,) = find_gaps(t, ids, rate, breaks)
    assert (gap["label"], gap["logging"], gap["note"]) == ("OFF", False, "")


# --- wheel order ---


def drift(n=400, dt=0.005, slip_deg=30.0, gas=1.0, speed_kmh=60.0):
    """Columns for n rows of a steady drift at heading 0: fronts gripping, rears spinning."""
    v = speed_kmh / 3.6
    travel = math.radians(slip_deg)   # at heading 0, slip = the direction of travel
    return {
        "t": [k * dt for k in range(n)],
        "gas": [gas] * n,
        "speed_kmh": [speed_kmh] * n,
        "vel_x": [v * math.sin(travel)] * n,
        "vel_z": [v * math.cos(travel)] * n,
        "heading": [0.0] * n,
        "wheel_slip_fl": [0.4 + 0.2 * (k % 2) for k in range(n)],
        "wheel_slip_fr": [0.4 + 0.2 * (k % 2) for k in range(n)],
        "wheel_slip_rl": [5.0 + 2.0 * (k % 2) for k in range(n)],
        "wheel_slip_rr": [5.0 + 2.0 * (k % 2) for k in range(n)],
    }


def test_wheel_medians():
    medians, seconds = wheel_medians(drift())
    assert medians == pytest.approx([0.5, 0.5, 6.0, 6.0])
    assert seconds == pytest.approx(399 * 0.005)


def test_front_spike_does_not_change_wheel_order():
    cols = drift()
    cols["wheel_slip_fr"][201] = 5e7
    assert statistics.mean(cols["wheel_slip_fr"]) > 6.0   # a mean would put this front above the rears
    medians, _ = wheel_medians(cols)
    assert medians == pytest.approx([0.5, 0.5, 6.0, 6.0])


@pytest.mark.parametrize(
    "change",
    [
        {"gas": 0.5},          # not on the gas
        {"slip_deg": 10.0},    # not sliding
        {"slip_deg": -10.0},
        {"speed_kmh": 3.0},    # below the slip-angle speed floor
    ],
)
def test_wheel_medians_only_judge_driving_on_the_gas_and_sliding(change):
    assert wheel_medians(drift(**change)) == ([None] * 4, 0)


def test_wheel_medians_leave_out_listed_breaks():
    _, seconds = wheel_medians(drift(), {200})
    assert seconds == pytest.approx(398 * 0.005)


def test_wheel_medians_count_sliding_either_way():
    medians, seconds = wheel_medians(drift(slip_deg=-30.0))
    assert medians == pytest.approx([0.5, 0.5, 6.0, 6.0])
    assert seconds > 1.0


# --- verdict ---


OK = dict(
    yaw_r=-0.9995, lat_r=0.9995, lon_r=0.998,
    steer_min=-0.98, steer_max=1.0,
    medians=[0.47, 0.47, 5.75, 5.72], wheel_s=70.8,
    logging_gaps=[], skip_share=0.01,
)


def test_verdict_ok():
    assert verdict(**OK) == []


@pytest.mark.parametrize(
    "change, words",
    [
        ({"yaw_r": 0.9}, "yaw"),
        ({"yaw_r": -0.94}, "yaw"),                 # |r| counts, whatever the sign
        ({"lat_r": 0.85}, "lateral"),
        ({"lon_r": -0.5}, "longitudinal"),
        ({"steer_max": 1.2}, "outside"),
        ({"steer_min": -1.1}, "outside"),
        ({"steer_min": -0.3}, "short of"),
        ({"steer_max": 0.4}, "short of"),
        ({"medians": [0.47, 6.0, 5.75, 5.72]}, "isn't above both fronts"),
        ({"medians": [5.72, 0.47, 5.75, 5.72]}, "isn't above both fronts"),   # a tie isn't above
        ({"wheel_s": 0.5, "medians": [None] * 4}, "0.5 s"),
        ({"logging_gaps": [0.2, 0.35]}, "2 logging gap(s) over 0.1 s, longest 0.350 s"),
        ({"skip_share": 0.84}, "84%"),
    ],
)
def test_verdict_each_failure_gives_its_reason(change, words):
    reasons = verdict(**{**OK, **change})
    assert len(reasons) == 1
    assert words in reasons[0]


def test_verdict_lists_every_reason():
    bad = dict(
        yaw_r=0.0, lat_r=0.0, lon_r=0.0, steer_min=-2.0, steer_max=0.2,
        medians=[None] * 4, wheel_s=0.0, logging_gaps=[0.2], skip_share=0.5,
    )
    # yaw, lateral, longitudinal, outside, short of, wheel order, gaps, skipping
    assert len(verdict(**bad)) == 8


# --- reading the CSV and JSON, and main ---


def csv_text(n):
    """A logger CSV of n rows, written the way the logger writes it."""
    out = io.StringIO()
    writer = csv.writer(out)
    writer.writerow(COLUMNS)
    for k in range(n):
        writer.writerow([round(k * TICK_S, 6), 1000 + k] + [0.5] * (len(COLUMNS) - 2))
    return out.getvalue()


def test_read_columns():
    cols, cut = read_columns(io.StringIO(csv_text(3)))
    assert not cut
    assert list(cols) == COLUMNS
    assert cols["t"] == [0.0, TICK_S, 2 * TICK_S]
    assert cols["packet_id"] == [1000, 1001, 1002]
    assert all(type(i) is int for i in cols["packet_id"])
    assert cols["wheel_slip_rr"] == [0.5, 0.5, 0.5]


@pytest.mark.parametrize(
    "cut_chars",
    [
        2,    # just the line ending: the row may be whole, but nothing says so
        3,    # inside the last number: "0." still parses, as 0.0 instead of 0.5
        20,   # mid-row: fields missing
        60,
    ],
)
def test_read_columns_leaves_out_a_cut_last_row(cut_chars):
    cols, cut = read_columns(io.StringIO(csv_text(3)[:-cut_chars]))
    assert cut
    assert cols["t"] == [0.0, TICK_S]
    assert all(len(column) == 2 for column in cols.values())


def test_read_columns_header_only():
    assert read_columns(io.StringIO(csv_text(0))) == ({name: [] for name in COLUMNS}, False)


def test_read_columns_empty_file():
    assert read_columns(io.StringIO("")) == ({}, False)


META = {"car": "hush_nissan_silvia_s14", "summary": {"breaks": 1, "break_intervals": []}}
META_BYTES = (json.dumps(META, indent=2) + "\n").encode()   # as the logger writes it


def test_read_meta():
    assert read_meta(META_BYTES) == META


@pytest.mark.parametrize(
    "data",
    [
        b"",                           # emptied by the rewrite, then killed
        META_BYTES[:40],               # cut off mid-write
        b"\x00" * 4096,                # zero bytes after a power cut
        b"\xff" + META_BYTES[1:],      # not text
    ],
)
def test_read_meta_damaged(data):
    assert read_meta(data) is None


def test_main_refuses_a_missing_file(capsys, tmp_path):
    with pytest.raises(SystemExit) as exc:
        main([str(tmp_path / "2026-10-11_143000.csv")])
    assert exc.value.code == 2
    assert "no such file" in capsys.readouterr().err


@pytest.mark.parametrize("name", ["2026-10-11_143000.json", "2026-10-11_143000"])
def test_main_refuses_anything_but_the_csv(capsys, tmp_path, name):
    # The file exists, so it's the suffix that's refused, before any reading.
    path = tmp_path / name
    path.write_bytes(META_BYTES)
    with pytest.raises(SystemExit) as exc:
        main([str(path)])
    assert exc.value.code == 2
    assert "give it the session's .csv" in capsys.readouterr().err
