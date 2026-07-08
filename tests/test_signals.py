"""
Tests for the pure signal math in driftscore.signals.

These run anywhere -- no Assetto Corsa, no Windows, no shared memory.
Every case encodes a fact we verified against live telemetry the hard way
(see the git history around the heading-convention fix): AC's `heading`
is mirrored yaw, so a car pointing toward atan2-direction d has
heading = -d.
"""

import math

import pytest

from driftscore.signals import slip_angle_deg


def velocity(travel_deg, speed=20.0, vy=0.0):
    """World velocity for a car MOVING toward `travel_deg` (atan2 convention)."""
    r = math.radians(travel_deg)
    return (speed * math.sin(r), vy, speed * math.cos(r))


def heading(pointing_deg):
    """AC heading (radians) for a car POINTING toward `pointing_deg`."""
    return -math.radians(pointing_deg)


@pytest.mark.parametrize("direction_deg", [0.0, 45.0, 90.0, -90.0, 135.0, -179.0])
def test_straight_line_reads_zero_in_any_direction(direction_deg):
    # Pointing exactly where it's moving -> zero slip, WHEREVER it points.
    # Direction-independence is the property both historical bugs violated.
    slip = slip_angle_deg(velocity(direction_deg), heading(direction_deg))
    assert slip == pytest.approx(0.0, abs=1e-6)


def test_matches_live_telemetry_snapshots():
    # Golden values recorded from a real AC session (2026-07-06). If the
    # heading convention ever regresses, these fail with numbers that
    # point straight at it.
    rolling = slip_angle_deg(velocity(79.0), math.radians(-85.2))
    donut = slip_angle_deg(velocity(84.4), math.radians(-46.5))
    straight = slip_angle_deg(velocity(170.9), math.radians(-172.1))
    assert rolling == pytest.approx(-6.2, abs=0.05)
    assert donut == pytest.approx(+37.9, abs=0.05)
    assert straight == pytest.approx(-1.2, abs=0.05)


def test_left_and_right_slides_are_symmetric():
    # Mirroring a slide left<->right must flip only the sign. Scoring
    # will rely on this symmetry.
    nose = heading(0.0)
    right = slip_angle_deg(velocity(+30.0), nose)
    left = slip_angle_deg(velocity(-30.0), nose)
    assert right == pytest.approx(+30.0, abs=1e-6)
    assert left == pytest.approx(-30.0, abs=1e-6)


def test_wraps_into_minus180_plus180():
    # Moving toward +170 with the nose at -20: the raw difference is
    # +190, which must wrap to -170 -- never leave [-180, 180].
    slip = slip_angle_deg(velocity(170.0), heading(-20.0))
    assert slip == pytest.approx(-170.0, abs=1e-6)


def test_vertical_velocity_is_ignored():
    # Same horizontal motion, big vertical component (uphill): identical
    # answer. Slip lives in the horizontal plane -- the "uphill bug"
    # turned out to be direction, not slope, and this keeps it that way.
    flat = slip_angle_deg(velocity(60.0), heading(45.0))
    uphill = slip_angle_deg(velocity(60.0, vy=15.0), heading(45.0))
    assert uphill == pytest.approx(flat, abs=1e-9)
