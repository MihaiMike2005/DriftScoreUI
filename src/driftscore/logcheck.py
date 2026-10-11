"""
The log check: is a logger recording sound, and which axis is which?

Run it on the AC PC right after a session, before recording more:

    python -m driftscore.logcheck data/raw/2026-10-11_143000.csv

It reads the CSV and, if there is one, the .json of the same name. It
reports the logging rate and every gap, finds the yaw, lateral and
longitudinal axes by correlation, checks the steering range and the wheel
order, and ends with OK or CHECK and the reasons. A broken struct shows
up here, not after days of recordings. Stdlib only, like the logger.
"""

import argparse
import bisect
import csv
import json
import math
import statistics
from pathlib import Path

from driftscore.signals import SLIP_SPEED_FLOOR_KMH, slip_angle_deg


G = 9.81
GAP_S = 0.1                  # rows further apart are a gap: printed, and never differenced across
DERIV_SPAN_S = 0.05          # derivatives over this span: row-to-row on the poll time is jittery (yaw r 0.98, not 0.999)
PAUSE_SHARE = 0.5            # a gap is a pause if packetId moved less than this share of what AC's rate predicts
MIN_YAW_R = 0.95             # best ang_vel axis against d(heading)/dt
MIN_ACC_R = 0.9              # best acc_g axis against the car-frame acceleration
STEER_LIMIT = 1.05           # steer is normalised to [-1, 1]; outside this the column is something else
STEER_REACH = 0.5            # a real session steers at least this far both ways
WHEEL_GAS = 0.8              # wheel order is judged while on the gas...
WHEEL_SLIP_DEG = 20          # ...and sliding, when the driven rears must spin fastest
MIN_WHEEL_S = 1.0            # less qualifying driving than this is too little to judge
MAX_SKIP_SHARE = 0.10        # more packetId steps of 2+ than this: the poll misses packets
WHEELS = ("fl", "fr", "rl", "rr")


def unwrap(angles) -> list:
    """Angles in radians with the jumps at ±π removed, so their derivative has no spikes."""
    out = []
    for a in angles:
        if out:
            a = out[-1] + (a - out[-1] + math.pi) % (2 * math.pi) - math.pi
        out.append(a)
    return out


def _segments(t, gap_s=GAP_S, breaks=()) -> list:
    """(start, end) index ranges, end exclusive, split wherever rows are over gap_s apart or a break ends."""
    cuts = [i for i in range(1, len(t)) if t[i] - t[i - 1] > gap_s or i in breaks]
    bounds = [0] + cuts + [len(t)]
    return list(zip(bounds, bounds[1:]))


def break_rows(t, break_intervals) -> set:
    """Indices of the rows that end a break the JSON lists, however short.

    The interval before each is left out of every number, as the logger
    leaves it out of its summary. Empty without break_intervals.
    """
    ends = {b["end_s"] for b in break_intervals or ()}
    return {i for i in range(1, len(t)) if t[i] in ends}


def derivative(t, x, span_s=DERIV_SPAN_S, gap_s=GAP_S, breaks=()) -> list:
    """dx/dt at each row, as a central difference over at least span_s.

    Each row uses the last row at least span_s/2 before it and the first at
    least span_s/2 after it. None where that would cross a gap over gap_s,
    or a break ending at a row in `breaks`, or run past either end of the
    data: a pause isn't a rate of change.
    """
    out = [None] * len(t)
    half = span_s / 2
    for start, end in _segments(t, gap_s, breaks):
        for i in range(start, end):
            lo = bisect.bisect_right(t, t[i] - half, start, end) - 1
            hi = bisect.bisect_left(t, t[i] + half, start, end)
            if lo >= start and hi < end:
                out[i] = (x[hi] - x[lo]) / (t[hi] - t[lo])
    return out


def pearson(a, b) -> float:
    """Pearson's r, by hand: statistics.correlation raises an error on a constant input.

    0 when either input is constant (or too short), so a dead axis reads
    as uncorrelated instead of crashing the check.
    """
    n = len(a)
    if n < 2 or min(a) == max(a) or min(b) == max(b):
        return 0.0
    ma, mb = sum(a) / n, sum(b) / n
    sab = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    saa = sum((x - ma) ** 2 for x in a)
    sbb = sum((y - mb) ** 2 for y in b)
    return sab / math.sqrt(saa * sbb)


def best_axis(candidates, reference) -> tuple:
    """(index, r, sign) of the candidate series that best tracks `reference`, by |r|.

    Rows where the reference is None (no derivative there) are left out.
    """
    rows = [i for i, v in enumerate(reference) if v is not None]
    ref = [reference[i] for i in rows]
    rs = [pearson([c[i] for i in rows], ref) for c in candidates]
    best = max(range(len(rs)), key=lambda k: abs(rs[k]))
    return best, rs[best], 1 if rs[best] >= 0 else -1


def car_frame(ax, az, heading) -> tuple:
    """(longitudinal, lateral) parts of a world x/z vector, for a car at `heading`.

    AC's heading turns the opposite way to atan2(x, z) (see
    signals.slip_angle_deg), so the nose points along (-sin h, cos h) and
    the side along (cos h, sin h).
    """
    s, c = math.sin(heading), math.cos(heading)
    return -ax * s + az * c, ax * c + az * s


def accel_refs(t, vel_x, vel_z, heading, breaks=()) -> tuple:
    """Longitudinal and lateral acceleration in g: d(velocity)/dt turned into the car frame.

    Not d(speed)/dt or speed x yaw rate: a drifting car's velocity isn't
    along its nose, and on the first session they gave r 0.40 and 0.78.
    """
    lon, lat = [], []
    dx, dz = derivative(t, vel_x, breaks=breaks), derivative(t, vel_z, breaks=breaks)
    for ax, az, h in zip(dx, dz, heading):
        if ax is None or az is None:
            lon.append(None)
            lat.append(None)
        else:
            a_lon, a_lat = car_frame(ax, az, h)
            lon.append(a_lon / G)
            lat.append(a_lat / G)
    return lon, lat


def step_stats(t, packet_ids, breaks=()) -> dict:
    """Rows/s, AC's packet rate, packetId step counts and the share of steps of 2+.

    Over intervals of GAP_S or less, and not before a row in `breaks`: gaps
    and breaks get their own report. AC's rate is packetId change over
    time, so it holds whatever our poll kept.

    The logger's summary leaves out only breaks, so it counts each logging
    gap's step, which this leaves out, and its rows/s is a little lower.
    Without logging gaps, the two step counts match.
    """
    steps = {"1": 0, "2": 0, "3+": 0, "reset": 0}
    intervals = 0
    rows_s = moved = packets_s = 0.0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt > GAP_S or i in breaks:
            continue
        step = packet_ids[i] - packet_ids[i - 1]
        steps["reset" if step < 1 else "3+" if step >= 3 else str(step)] += 1
        intervals += 1
        rows_s += dt
        if step >= 1:   # a reset says nothing about the rate
            moved += step
            packets_s += dt
    kept = intervals - steps["reset"]
    return {
        "rows_per_s": intervals / rows_s if rows_s > 0 else 0.0,
        "packet_rate": moved / packets_s if packets_s > 0 else 0.0,
        "steps": steps,
        "skip_share": (steps["2"] + steps["3+"]) / kept if kept else 0.0,
    }


def classify_gap(dt, step, rate, statuses=None, have_breaks=False) -> tuple:
    """(label, logging, note) for one gap of dt seconds where packetId moved `step`.

    `statuses` are the AC statuses the JSON lists for a break ending here.
    `have_breaks` says the JSON lists breaks at all, so the logger saw AC
    live through any gap it doesn't list. Otherwise the packetId test
    decides: AC freezes packetId while paused, so a pause moves it far
    less than `rate` predicts. Only a logging gap counts toward the
    verdict; `note` says where the JSON and the packetId test disagree.
    """
    predicted = rate * dt
    by_packets = "pause" if step < PAUSE_SHARE * predicted else "logging gap"
    moved = f"packetId moved {step}, the rate predicts {predicted:.0f}"
    if statuses:
        label = " + ".join(statuses)
        # A short break's prediction is a few packets, too few to tell.
        if by_packets == "logging gap" and dt > GAP_S:
            return label, False, f"the JSON says {label}, but {moved}: it kept counting"
        return label, False, ""
    if have_breaks and by_packets == "pause":
        return by_packets, False, f"the JSON says AC was live, but {moved}: AC stalled"
    return by_packets, by_packets == "logging gap", ""


def find_gaps(t, packet_ids, rate, break_intervals=None) -> list:
    """Every interval over GAP_S, and every break the JSON lists however short.

    Dicts of start_s, end_s, step, label, logging and note; see
    classify_gap. `break_intervals` None (no JSON, or one from before the
    logger saved them) leaves the packetId test alone.
    """
    by_end = {b["end_s"]: b["statuses"] for b in break_intervals or ()}
    gaps = []
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        statuses = by_end.get(t[i])
        if dt <= GAP_S and not statuses:
            continue
        step = packet_ids[i] - packet_ids[i - 1]
        label, logging, note = classify_gap(dt, step, rate, statuses, break_intervals is not None)
        gaps.append({
            "start_s": t[i - 1], "end_s": t[i], "step": step,
            "label": label, "logging": logging, "note": note,
        })
    return gaps


def wheel_medians(cols, breaks=()) -> tuple:
    """(median slip per wheel FL, FR, RL, RR, seconds judged), while on the gas and sliding.

    Medians, not means: front slip spikes to ~5e7 at big angles and
    swamps a mean. Below SLIP_SPEED_FLOOR_KMH the slip angle is noise, so
    those rows don't count. Medians are None when no row qualifies. The
    seconds leave out gaps, and breaks ending at a row in `breaks`.
    """
    t = cols["t"]
    rows = [
        i for i in range(len(t))
        if cols["gas"][i] > WHEEL_GAS
        and cols["speed_kmh"][i] >= SLIP_SPEED_FLOOR_KMH
        and abs(slip_angle_deg((cols["vel_x"][i], 0.0, cols["vel_z"][i]), cols["heading"][i])) > WHEEL_SLIP_DEG
    ]
    seconds = sum(
        t[i + 1] - t[i] for i in rows
        if i + 1 < len(t) and t[i + 1] - t[i] <= GAP_S and i + 1 not in breaks
    )
    if not rows:
        return [None] * len(WHEELS), seconds
    return [statistics.median(cols[f"wheel_slip_{w}"][i] for i in rows) for w in WHEELS], seconds


def verdict(yaw_r, lat_r, lon_r, steer_min, steer_max, medians, wheel_s, logging_gaps, skip_share) -> list:
    """The reasons to CHECK the recording; empty means OK.

    `logging_gaps` holds the length in seconds of each logging gap.
    """
    reasons = []
    if abs(yaw_r) < MIN_YAW_R:
        reasons.append(f"yaw: best |r| is {abs(yaw_r):.3f}, under {MIN_YAW_R}")
    for name, r in (("lateral", lat_r), ("longitudinal", lon_r)):
        if abs(r) < MIN_ACC_R:
            reasons.append(f"{name}: best |r| is {abs(r):.3f}, under {MIN_ACC_R}")
    steer = f"steer runs {steer_min:.2f} to {steer_max:.2f}"
    if steer_min < -STEER_LIMIT or steer_max > STEER_LIMIT:
        reasons.append(f"{steer}, outside ±{STEER_LIMIT}")
    if steer_min > -STEER_REACH or steer_max < STEER_REACH:
        reasons.append(f"{steer}, short of ±{STEER_REACH}")
    if wheel_s < MIN_WHEEL_S:
        reasons.append(f"wheel order: {wheel_s:.1f} s of driving on the gas and sliding, under {MIN_WHEEL_S} s")
    elif min(medians[2:]) <= max(medians[:2]):
        slips = "  ".join(f"{w.upper()} {m:.2f}" for w, m in zip(WHEELS, medians))
        reasons.append(f"wheel order: the rears' median slip isn't above both fronts ({slips})")
    if logging_gaps:
        reasons.append(f"{len(logging_gaps)} logging gap(s) over {GAP_S} s, longest {max(logging_gaps):.3f} s")
    if skip_share > MAX_SKIP_SHARE:
        reasons.append(
            f"{skip_share:.0%} of packetId steps are 2 or more (limit {MAX_SKIP_SHARE:.0%}): "
            "the poll is too slow to keep every packet"
        )
    return reasons


def read_columns(lines) -> tuple:
    """(columns by name, cut) from the lines of a logger CSV: the open file, or any iterable.

    The logger ends every row with a newline, so a last line without one
    was cut off mid-row: the logger was stopped without Ctrl+C (window
    closed, a crash). That row is left out, and `cut` says so. Counting its
    fields wouldn't do: a cut inside the last number still parses.

    Rows go straight into the columns, one at a time: a dict per row came
    to 1.2 GB for a 20-minute session.
    """
    cut = False

    def whole(lines):
        nonlocal cut
        for line in lines:
            if not line.endswith("\n"):   # only the last line can lack it
                cut = True
                return
            yield line

    reader = csv.reader(whole(lines))
    header = next(reader, [])
    cols = {name: [] for name in header}
    parsers = [(cols[name], int if name == "packet_id" else float) for name in header]
    for row in reader:
        for (column, parse), value in zip(parsers, row):
            column.append(parse(value))
    return cols, cut


def read_meta(data) -> dict | None:
    """The session JSON from its bytes, or None if it's damaged.

    The logger rewrites the JSON when Ctrl+C stops it, and writing empties
    the file first. Killed during that write, or by a power cut just after,
    it leaves the file empty, cut off or full of zero bytes.
    """
    try:
        return json.loads(data)
    except ValueError:   # JSONDecodeError, or UnicodeDecodeError on bytes that aren't text
        return None


def report(csv_path) -> list:
    """Print the check for one recording; returns verdict's reasons."""
    with open(csv_path, newline="", encoding="utf-8") as f:
        cols, cut = read_columns(f)
    json_path = Path(csv_path).with_suffix(".json")
    meta = read_meta(json_path.read_bytes()) if json_path.exists() else {}
    summary = (meta or {}).get("summary")
    break_intervals = (summary or {}).get("break_intervals")

    t, ids = cols.get("t", []), cols.get("packet_id", [])
    if len(t) < 2:
        print(f"{csv_path}: {len(t)} row(s), nothing to check.")
        return ["fewer than 2 rows"]

    print(f"{csv_path}: {len(t)} rows over {t[-1] - t[0]:.1f} s")
    if cut:
        print("The last row was cut off (the logger was stopped without Ctrl+C), so it's left out.")
    if meta is None:
        print(
            "The JSON is damaged (the logger was stopped while writing it): "
            "gaps are labelled by the packetId test alone."
        )
    elif not meta:
        print("No JSON next to it: gaps are labelled by the packetId test alone.")
    else:
        print(f"Car: {meta.get('car')}   Track: {meta.get('track')}   poll: {meta.get('poll_ms')} ms")
        if summary is None:
            # The logger writes the summary when Ctrl+C stops it.
            print(
                "The JSON has no summary (the logger was stopped without Ctrl+C): "
                "gaps are labelled by the packetId test alone."
            )
        elif break_intervals is None:
            print(
                f"The JSON counts {summary.get('breaks', '?')} break(s) without their times "
                "(older logger): gaps are labelled by the packetId test alone."
            )

    breaks = break_rows(t, break_intervals)
    stats = step_stats(t, ids, breaks)
    steps = "  ".join(f"{k}: {v}" for k, v in stats["steps"].items())
    print("\nRate")
    print(f"  rows/s         {stats['rows_per_s']:.1f}")
    print(f"  AC packets/s   {stats['packet_rate']:.1f}")
    print(f"  packetId steps {steps}   (2+: {stats['skip_share']:.1%})")

    gaps = find_gaps(t, ids, stats["packet_rate"], break_intervals)
    logging_gaps = [g["end_s"] - g["start_s"] for g in gaps if g["logging"]]
    print(f"\nGaps over {GAP_S} s and breaks: {len(gaps)}, logging gaps: {len(logging_gaps)}")
    for g in gaps:
        print(
            f"  {g['start_s']:9.3f} -> {g['end_s']:9.3f} s  ({g['end_s'] - g['start_s']:.3f} s)  "
            f"packetId +{g['step']}  {g['label']}"
        )
        if g["note"]:
            print(f"    note: {g['note']}")

    heading = unwrap(cols["heading"])
    yaw_ref = derivative(t, heading, breaks=breaks)
    lon_ref, lat_ref = accel_refs(t, cols["vel_x"], cols["vel_z"], cols["heading"], breaks)
    ang_vel = [cols[f"ang_vel_{a}"] for a in "xyz"]
    acc_g = [cols[f"acc_g_{a}"] for a in "xyz"]
    print()
    found = {}
    for name, cands, prefix, ref in (
        ("yaw", ang_vel, "ang_vel", yaw_ref),
        ("lateral", acc_g, "acc_g", lat_ref),
        ("longitudinal", acc_g, "acc_g", lon_ref),
    ):
        axis, r, sign = best_axis(cands, ref)
        found[name] = r
        column = f"{prefix}_{'xyz'[axis]}"
        print(f"{name.capitalize():<13} {column:<10} r {r:+.3f}   sign {'+' if sign > 0 else '-'}")

    steer_min, steer_max = min(cols["steer"]), max(cols["steer"])
    print(f"Steer         {steer_min:.3f} to {steer_max:.3f}")

    medians, wheel_s = wheel_medians(cols, breaks)
    if medians[0] is None:
        print("Wheel slip    no driving on the gas and sliding")
    else:
        slips = "  ".join(f"{w.upper()} {m:.2f}" for w, m in zip(WHEELS, medians))
        print(
            f"Wheel slip    {slips}   (median over {wheel_s:.1f} s with gas > {WHEEL_GAS} "
            f"and |slip| > {WHEEL_SLIP_DEG}°)"
        )

    reasons = verdict(
        found["yaw"], found["lateral"], found["longitudinal"], steer_min, steer_max,
        medians, wheel_s, logging_gaps, stats["skip_share"],
    )
    print("\nVerdict: " + ("CHECK" if reasons else "OK"))
    for reason in reasons:
        print(f"  - {reason}")
    return reasons


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="python -m driftscore.logcheck",
        description="Check a logger recording: rate, gaps, axes, steering, wheel order.",
    )
    parser.add_argument("csv", help="the session's .csv; its .json is read from beside it")
    args = parser.parse_args(argv)
    # Tab completion stops at the stem the .csv and .json share, so the
    # .json, or no suffix at all, is an easy slip.
    if Path(args.csv).suffix.lower() != ".csv":
        parser.error(f"give it the session's .csv, not {args.csv}")
    if not Path(args.csv).is_file():
        parser.error(f"no such file: {args.csv}")
    report(args.csv)


if __name__ == "__main__":
    main()
