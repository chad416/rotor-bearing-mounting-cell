"""Model verification: each model against an independent result (DOSSIER §10.2)."""
import math

import numpy as np
import pytest

from mount.fits import (Ring, lame_pressure, operating_clearance_mc, press_force, raceway_ratio, smoothing_um,
                        tolerance_interference)
from mount.plant import load_config, move_time, ring_of
from mount.thermal import heating_time_s, simulate_hot_mount

CFG = load_config()
E, NU = CFG["steel"]["E"], CFG["steel"]["nu"]


def test_tolerance_stack_matches_the_catalogue_limits():
    """m5 shaft (+13/+28 um) on a normal-class bore (0/-20 um): 13 to 48 um."""
    assert tolerance_interference(CFG["fits"]["shaft"]["m5_80_120"], CFG["fits"]["bore_dmp"]["80_120"]) == (13, 48)


def test_lame_pressure_matches_the_general_two_material_formula():
    """General press-fit relation (any E, nu), here with equal materials and a solid shaft:
    delta = d p [ (1/E)((b2 + a2)/(b2 - a2) + nu) + (1/E)(1 - nu) ]."""
    r = ring_of(CFG, "DE")
    delta = 30.0
    p = float(lame_pressure(delta, r, E))
    a, b, d = r.d / 2, r.d1 / 2, r.d * 1e-3
    back = d * p * ((1 / E) * ((b * b + a * a) / (b * b - a * a) + NU) + (1 / E) * (1 - NU))
    assert back * 1e6 == pytest.approx(delta, rel=1e-12)


def test_raceway_growth_from_the_lame_displacement_field():
    """Plane-stress Lame solution of the ring alone: sigma_r = A - B/r^2 with sigma_r(a) = -p, sigma_r(b) = 0,
    u(r) = r/E [(1 - nu) A + (1 + nu) B / r^2]; the outer diameter grows by 2 u(b). Compare with f1 = a/b."""
    r = ring_of(CFG, "DE")
    a, b = r.d / 2 * 1e-3, r.d1 / 2 * 1e-3
    delta = 40.0
    p = float(lame_pressure(delta, r, E))
    bb = p * a * a * b * b / (b * b - a * a)
    aa = bb / (b * b)
    u_b = b / E * ((1 - NU) * aa + (1 + NU) * bb / (b * b))
    assert 2 * u_b * 1e6 / delta == pytest.approx(raceway_ratio(r), rel=1e-9)


def test_press_force_is_linear_in_engagement_and_zero_without_interference():
    r = ring_of(CFG, "DE")
    f1 = float(press_force(10, 30, r, E, 0.15))
    f2 = float(press_force(20, 30, r, E, 0.15))
    assert f2 == pytest.approx(2 * f1)
    assert float(press_force(20, -5, r, E, 0.15)) == 0.0


def test_clearance_monte_carlo_without_spread_reproduces_the_worst_case():
    r = ring_of(CFG, "DE")
    fc = dict(CFG["fits"], sigma_share=1e-9)
    shaft, bore = [28, 28], [-20, -20]
    mc = operating_clearance_mc(r, shaft, bore, [36, 36], fc, n=1000)
    expected = 36 - raceway_ratio(r) * (48 - smoothing_um(fc)) - 0.012 * fc["dt_inner_outer_k"] * r.dm
    assert mc["mean"] == pytest.approx(expected, abs=1e-6)


def test_gap_at_the_seat_start_is_the_free_thermal_expansion():
    """No transfer time and a fast slide: the first gap is (alpha d dT - delta)/2."""
    r = ring_of(CFG, "DE")
    res = simulate_hot_mount(r, 30.0, 80.0, 0.0, 2000.0, CFG, t_after=1.0)
    expected = 0.5 * (CFG["steel"]["alpha"] * r.d * 1e3 * 80.0 - 30.0)
    assert res.gap_at_seat_start_um == pytest.approx(expected, abs=0.3)


def test_transfer_cooling_is_newton_cooling_without_balls_and_radiation():
    cfg = {**CFG, "mounting": {**CFG["mounting"], "ball_conductance_w_k": 0.0}, "air": {**CFG["air"], "emissivity": 0.0}}
    r = ring_of(CFG, "DE")
    t_tr = 600.0
    res = simulate_hot_mount(r, 30.0, 80.0, t_tr, 2000.0, cfg, t_after=0.5)
    d = r.d * 1e-3
    area = math.pi * d * r.B * 1e-3 + 2 * math.pi / 4 * ((r.d1 * 1e-3) ** 2 - d ** 2)
    tau = r.inner_ring_mass(CFG["steel"]["rho"]) * CFG["steel"]["cp"] / (CFG["air"]["h_transfer"] * area)
    assert res.t_ring_at_seat_c == pytest.approx(20 + 80 * math.exp(-t_tr / tau), abs=0.3)


def test_hotter_ring_slides_with_more_margin_but_grips_later():
    r = ring_of(CFG, "DE")
    a = simulate_hot_mount(r, 48.0, 60.0, 2.0, 20.0, CFG)
    b = simulate_hot_mount(r, 48.0, 90.0, 2.0, 20.0, CFG)
    assert b.min_gap_um > a.min_gap_um and b.t_full_grip_s > a.t_full_grip_s


def test_too_cold_ring_seizes_on_the_seat():
    r = ring_of(CFG, "DE")
    assert simulate_hot_mount(r, 48.0, 35.0, 2.0, 20.0, CFG).seized


def test_heating_energy_balance():
    r = ring_of(CFG, "DE")
    t = heating_time_s(r, 90.0, CFG)
    assert t * CFG["heater"]["power_w"] * CFG["heater"]["efficiency"] == pytest.approx(r.mass * CFG["steel"]["cp"] * 90.0)


def test_trapezoidal_move_time():
    assert move_time(1600, 1500, 4000) == pytest.approx(1600 / 1500 + 1500 / 4000)
    assert move_time(100, 1500, 4000) == pytest.approx(2 * math.sqrt(100 / 4000))


def test_payload_within_the_robot_rating():
    m = max(CFG["bearings"][p]["mass"] for p in ("DE", "NDE")) + CFG["robot"]["gripper_kg"]
    assert m <= CFG["robot"]["payload_kg"]
