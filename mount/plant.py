"""Closed loop of one bearing mount: the compiled ST controller MOUNT_CELL against models of the induction heater,
the robot (trapezoidal moves of BRG_MOUNT.ls), the hot slide-on (mount/thermal.py), the hold-down clamp and the servo
press (Lame force, mount/fits.py)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from mount.fits import Ring, press_force, smoothing_um, tolerance_interference
from mount.thermal import simulate_hot_mount

ROOT = Path(__file__).resolve().parents[1]


def load_config() -> dict:
    return yaml.safe_load((ROOT / "config" / "cell.yaml").read_text(encoding="utf-8"))


def move_time(dist_mm: float, v: float, a: float) -> float:
    """Trapezoidal (or triangular) velocity profile."""
    if dist_mm <= v * v / a:
        return 2 * math.sqrt(dist_mm / a)
    return dist_mm / v + v / a


def ring_of(cfg: dict, pos: str) -> Ring:
    return Ring.from_config(cfg["bearings"][pos], cfg["bearings"]["d1_share"])


def fit_of(pos: str) -> str:
    return "m5_80_120" if pos == "DE" else "k5_80_120"


@dataclass
class Scenario:
    delta_um: float | None = None          # actual interference (default: tolerance maximum = worst for sliding)
    mu: float | None = None                # actual press friction (default: middle of the range)
    transfer_delay_s: float = 0.0          # robot stopped during the transfer (light curtain, operator)
    stop_mid_s: float = 0.0                # protective stop with the ring half way over the seat
    hold_s: float | None = None            # per-part hold time (gauged interference, M5)
    v_insert: float = 20.0                 # mm/s
    t_shaft: float | None = None
    seat_blocked_mm: float | None = None   # press: an obstacle (chip, cocked ring) stops the stroke this short of the seat
    over: dict = field(default_factory=dict)


def recipe(cfg: dict, alt: dict, pos: str, limits: dict) -> dict:
    """limits: per bearing position, from scripts/run_studies.py (process window + press window)."""
    lim = limits[pos]
    r = dict(rcMethod={"hot": 1 if alt["hold"] == "clamp" else 0, "press": 2}[alt["method"]],
             rcDeltaT=alt["dT"], rcTMax=cfg["mounting"]["t_bearing_max"], rcTtrMax=lim.get("t_tr_max", 60.0),
             rcInsertMax=lim.get("insert_max_s", 1.5 * (cfg["bearings"][pos]["B"] + cfg["mounting"]["seat_lead_mm"]) / 5.0), rcHoldS=lim.get("hold_s", 0.0), rcMaxReheats=1,
             rcSeatPos=cfg["bearings"][pos]["B"], rcPosTol=0.3, rcFSeatRise=5.0, rcFMax=cfg["press"]["f_rated_kn"])
    if alt["method"] == "press":
        w = lim["press_window"]
        r.update(rcX1=w["x1"], rcX2=w["x2"], rcF1Lo=w["f1"][0], rcF1Hi=w["f1"][1], rcF2Lo=w["f2"][0], rcF2Hi=w["f2"][1])
    else:
        r.update(rcX1=0.0, rcX2=0.0, rcF1Lo=0.0, rcF1Hi=0.0, rcF2Lo=0.0, rcF2Hi=0.0)
    return r


def run_mount(plc, cfg: dict, alt_key: str, pos: str, limits: dict, sc: Scenario | None = None, dt: float = 0.05,
              t_max: float = 900.0, keep_fx: bool = False) -> dict:
    sc = sc or Scenario()
    alt = cfg["alternatives"][alt_key]
    ring = ring_of(cfg, pos)
    st, mo, rb, pr = cfg["steel"], cfg["mounting"], cfg["robot"], cfg["press"]
    dmin, dmax = tolerance_interference(cfg["fits"]["shaft"][fit_of(pos)], cfg["fits"]["bore_dmp"]["80_120"])
    delta = sc.delta_um if sc.delta_um is not None else dmax
    d_eff = max(delta - smoothing_um(cfg["fits"]), -50.0)
    mu = sc.mu if sc.mu is not None else float(np.mean(mo["mu"]))
    t_shaft = mo["t_shaft"] if sc.t_shaft is None else sc.t_shaft
    v, a = rb["v_tcp_mm_s"], rb["a_tcp_mm_s2"]
    p = rb["path"]
    t_pick_seq = move_time(p["heater_approach"], v, a) * 2 + rb["t_grip_s"]           # down, grip, up
    t_transfer_nom = move_time(p["heater_to_shaft"], v, a) + move_time(p["shaft_approach"], 300.0, a)
    t_retract = rb["t_grip_s"] + move_time(p["retract"], 1000.0, a) + move_time(p["back_to_heater"], v, a)
    # ---- start
    plc.reset()
    rec = recipe(cfg, alt, pos, limits)
    if sc.hold_s is not None:
        rec["rcHoldS"] = sc.hold_s
    plc.set(**rec, cycleTime=dt, safetyOk=True, rbReady=True, mTShaft=t_shaft,
            mTBearing=mo["t_ambient"], cmdStart=True)
    plc.scan()
    plc.set(cmdStart=False)
    t = 0.0
    tb = mo["t_ambient"]                    # bearing (inner ring) temperature on the heater
    c_b = ring.mass * st["cp"]
    robot = "idle"
    r_t = 0.0                               # time left in the current robot action
    picked_at = None
    at_shaft_at = None
    seat_at = None
    hot = None
    clamp_t = None
    press_x = None
    press_t0 = None
    t_heat = 0.0
    robot_busy = 0.0
    out = {}
    fx = {"x": [], "f": []}
    events = {}
    while t < t_max:
        o = plc.outputs()
        state = o["STATE"]
        # ---- heater
        if o["QHEATERON"]:
            tb += cfg["heater"]["power_w"] * cfg["heater"]["efficiency"] / c_b * dt
            t_heat += dt
        # ---- robot
        if robot not in ("idle", "done", "stalled"):
            robot_busy += dt
        if robot == "idle" and o["QRBPICK"]:
            robot, r_t = "picking", t_pick_seq
        elif robot == "picking":
            r_t -= dt
            if r_t <= 0:
                robot, r_t = "transfer", t_transfer_nom + (sc.transfer_delay_s if not events.get("delay_used") else 0.0)
                events["delay_used"] = True                 # a one-off stop: the re-pick after a re-heat is normal
                picked_at = t
                t_at_pick = tb
                plc.set(rbPicked=True, rbReady=False)
        elif robot == "transfer":
            r_t -= dt
            if r_t <= 0:
                robot = "at_shaft"
                at_shaft_at = t
                plc.set(rbAtShaft=True)
        elif robot == "at_shaft":
            if o["QRBABORT"]:
                robot, r_t = "returning", t_transfer_nom + rb["t_grip_s"]
            elif o["QRBINSERT"]:
                # the hot slide-on physics, with the real transfer time and bearing temperature
                t_tr = t - picked_at
                hot = simulate_hot_mount(ring, d_eff + smoothing_um(cfg["fits"]), t_at_pick - t_shaft, t_tr,
                                         sc.v_insert, cfg, t_shaft=t_shaft, stop_mid_s=sc.stop_mid_s)
                robot, r_t = "inserting", (ring.B + mo["seat_lead_mm"]) / sc.v_insert + sc.stop_mid_s
            elif o["QRBRELEASE"]:                    # press methods: leave the ring on the lead-in
                robot, r_t = "leaving", t_retract
                plc.set(rbReleased=True)
        elif robot == "inserting":
            r_t -= dt
            if r_t <= 0 and not hot.seized:
                robot = "holding"
                seat_at = t
                plc.set(rbSeated=True)
            elif r_t <= 0 and hot.seized:
                robot = "stalled"                    # the ring stopped on the seat: no face signal
        elif robot == "holding":
            if o["QRBRELEASE"]:
                robot, r_t = "leaving", t_retract
                events["robot_release_after_seat_s"] = t - seat_at
                plc.set(rbReleased=True)
        elif robot == "returning":
            r_t -= dt
            if r_t <= 0:
                tb = tb - 0.0                        # cooling on the way back is in the re-heat time
                robot = "idle"
                plc.set(rbReturned=True, rbPicked=False, rbAtShaft=False, rbReady=True)
        elif robot == "leaving":
            r_t -= dt
            if r_t <= 0:
                robot = "done"
        if robot == "idle" and not o["QRBABORT"]:
            plc.set(rbReturned=False)
        # bearing on the robot cools (lumped; the hot-mount model does this in detail)
        if robot in ("transfer", "at_shaft") and alt["method"] == "press":
            tb -= cfg["air"]["h_transfer"] * math.pi * ring.D * 1e-3 * ring.B * 1e-3 * 3 * (tb - mo["t_ambient"]) / c_b * dt
        # ---- clamp
        if o["QCLAMP"]:
            clamp_t = (clamp_t or 0.0) + dt
            plc.set(mClampClosed=clamp_t >= 1.5)
        # ---- press
        if o["QPRESSGO"]:
            if press_t0 is None:
                press_t0 = t
                ring_t_press = tb
            tp = t - press_t0 - pr["approach_s"]
            x = max(0.0, tp * pr["v_mm_s"])
            # warm ring: the interference grows back as the ring cools on the shaft (lumped, contact conductance)
            dT_ring = max(ring_t_press - t_shaft, 0.0) * math.exp(-max(tp, 0.0) / 12.0)
            d_now = d_eff - st["alpha"] * ring.d * 1e3 * dT_ring
            stop = ring.B - (sc.seat_blocked_mm or 0.0)
            f = float(press_force(min(x, stop), d_now, ring, st["E"], mu)) / 1e3
            if x >= stop:
                f += 50.0 * (x - stop)               # shoulder (or obstacle) stiffness 50 kN/mm [A]
                x_plot = stop + (x - stop) * 0.02
            else:
                x_plot = x
            done = x >= stop and f >= float(press_force(stop, d_now, ring, st["E"], mu)) / 1e3 + 15.0
            plc.set(mPressPos=x_plot, mPressForce=f, mPressDone=done)
            if keep_fx:
                fx["x"].append(x_plot)
                fx["f"].append(f)
        plc.set(mTBearing=tb)
        plc.scan()
        t += dt
        if plc.get("STATE") in (8, 9) and robot in ("done", "stalled", "idle", "holding", "at_shaft", "leaving"):
            if robot == "leaving":
                continue
            break
    out.update({"alt": alt_key, "pos": pos, "verdict": int(plc.get("VERDICT")), "nok": int(plc.get("NOKCODE")),
                "state": int(plc.get("STATE")), "t_cycle": t, "t_heat": t_heat, "robot_busy": robot_busy,
                "t_transfer_plc": float(plc.get("TTRANSFER")), "t_hold_plc": float(plc.get("THOLD")),
                "reheats": int(plc.get("REHEATS")), "f_x1": float(plc.get("FATX1")), "f_x2": float(plc.get("FATX2")),
                "f_max": float(plc.get("FMAXSEEN")), "delta_um": delta, "mu": mu})
    if hot is not None:
        out.update({"seized": hot.seized, "min_gap_um": hot.min_gap_um, "t_full_grip": hot.t_full_grip_s,
                    "t_first_grip": hot.t_first_grip_s,
                    "seat_held_until_full_grip": bool(hot.t_full_grip_s is not None and out["t_hold_plc"] >= hot.t_full_grip_s - 1e-9)})
    out.update(events)
    if keep_fx:
        out["fx"] = fx
    return out
