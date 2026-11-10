"""All studies of DOSSIER §6-§9 -> results/studies.json (+ results/traces.json for the figures).

    bash scripts/build_plc.sh && python -m scripts.run_studies [--quick]
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import time
from pathlib import Path

import numpy as np

from mount.fits import (hoop_stress_bore, lame_pressure, operating_clearance_mc, press_force, raceway_ratio,
                        smoothing_um, tolerance_interference)
from mount.plant import Scenario, fit_of, load_config, move_time, ring_of, run_mount
from mount.plc import Plc
from mount.thermal import heating_time_s, simulate_hot_mount

ROOT = Path(__file__).resolve().parents[1]
POS = ("DE", "NDE")


def clearance_range(cfg, pos):
    return cfg["fits"]["clearance"]["100_120" if cfg["bearings"][pos]["d"] > 100 else "80_100"]


def fits_study(cfg) -> dict:
    out = {}
    bore = cfg["fits"]["bore_dmp"]["80_120"]
    E = cfg["steel"]["E"]
    opts = {"DE": [("k5", "C3"), ("m5", "C3"), ("m6", "C3"), ("n6", "C3"), ("m5", "CN"), ("m6", "CN")],
            "NDE": [("k5", "C3"), ("m5", "C3"), ("k5", "CN"), ("m5", "CN")]}
    for pos in POS:
        ring = ring_of(cfg, pos)
        rows = []
        for sh, cl in opts[pos]:
            shaft = cfg["fits"]["shaft"][f"{sh}_80_120"]
            dmin, dmax = tolerance_interference(shaft, bore)
            mc = operating_clearance_mc(ring, shaft, bore, clearance_range(cfg, pos)[cl], cfg["fits"])
            rows.append({"shaft": sh, "clearance": cl, "delta_min": dmin, "delta_max": dmax,
                         "delta_eff_min": dmin - smoothing_um(cfg["fits"]), **mc,
                         "p_max_mpa": float(lame_pressure(dmax, ring, E)) / 1e6,
                         "hoop_max_mpa": float(hoop_stress_bore(dmax, ring, E)) / 1e6,
                         "f_press_max_kn": float(press_force(ring.B, dmax, ring, E, cfg["mounting"]["mu"][1])) / 1e3})
        dt_sweep = []
        rec = ("m5" if pos == "DE" else "k5", "C3")
        for dtk in (0.0, 2.0, 3.0, 3.5, 4.0, 4.5, 5.0, 7.5, 10.0):
            mc = operating_clearance_mc(ring, cfg["fits"]["shaft"][f"{rec[0]}_80_120"], bore, clearance_range(cfg, pos)["C3"],
                                        cfg["fits"], dt_k=dtk)
            dt_sweep.append({"dt_k": dtk, "p_negative": mc["p_negative"], "mean": mc["mean"]})
        out[pos] = {"ring": {"d": ring.d, "D": ring.D, "B": ring.B, "d1": ring.d1, "f1": raceway_ratio(ring),
                             "inner_ring_kg": ring.inner_ring_mass(cfg["steel"]["rho"]), "mass": ring.mass},
                    "smoothing_um": smoothing_um(cfg["fits"]), "options": rows, "recommended": list(rec), "dt_sweep": dt_sweep}
    return out


def t_transfer_nominal(cfg) -> float:
    rb = cfg["robot"]
    return move_time(rb["path"]["heater_to_shaft"], rb["v_tcp_mm_s"], rb["a_tcp_mm_s2"]) + \
        move_time(rb["path"]["shaft_approach"], 300.0, rb["a_tcp_mm_s2"])


def process_window(cfg, quick=False) -> dict:
    """Hot slide-on at the tolerance maximum (the tightest case for sliding) and grip time at both tolerance ends."""
    gap_min = cfg["limits"]["min_gap_margin_um"]
    bore = cfg["fits"]["bore_dmp"]["80_120"]
    out = {}
    dts = [40, 50, 60, 70, 80, 90, 100] if not quick else [60, 90]
    for pos in POS:
        ring = ring_of(cfg, pos)
        dmin, dmax = tolerance_interference(cfg["fits"]["shaft"][fit_of(pos)], bore)
        rows = []
        for dT in dts:
            def margin(t_tr, v=20.0):
                return simulate_hot_mount(ring, dmax, dT, t_tr, v, cfg).min_gap_um

            lo, hi = 0.0, 300.0
            if margin(0.0) < gap_min:
                t_tr_max = 0.0
            else:
                while hi - lo > 0.5:
                    mid = 0.5 * (lo + hi)
                    lo, hi = (mid, hi) if margin(mid) >= gap_min else (lo, mid)
                t_tr_max = lo
            tn = t_transfer_nominal(cfg)
            if simulate_hot_mount(ring, dmax, dT, tn, 20.0, cfg).seized:
                t_stop_max = 0.0
            else:
                lo, hi = 0.0, 120.0
                while hi - lo > 0.25:
                    mid = 0.5 * (lo + hi)
                    lo, hi = (mid, hi) if not simulate_hot_mount(ring, dmax, dT, tn, 20.0, cfg, stop_mid_s=mid).seized else (lo, mid)
                t_stop_max = lo
            r_max = simulate_hot_mount(ring, dmax, dT, tn, 20.0, cfg)
            r_min = simulate_hot_mount(ring, dmin, dT, tn, 20.0, cfg)
            r_slow = simulate_hot_mount(ring, dmax, dT, tn, 5.0, cfg)
            rows.append({"dT": dT, "t_tr_max": t_tr_max, "t_stop_mid_max": t_stop_max, "min_gap_nominal": r_max.min_gap_um,
                         "min_gap_slow_5mm_s": r_slow.min_gap_um, "seized_slow": r_slow.seized,
                         "t_full_grip_dmax": r_max.t_full_grip_s, "t_full_grip_dmin": r_min.t_full_grip_s,
                         "t_first_grip_dmax": r_max.t_first_grip_s, "t_first_grip_dmin": r_min.t_first_grip_s,
                         "face_gap_first_grip_dmin": r_min.face_gap_if_released_um,
                         "face_gap_first_grip_dmax": r_max.face_gap_if_released_um,
                         "grip_node_dmax": r_max.grip_node_from_face,
                         "t_heat_s": heating_time_s(ring, dT, cfg), "t_ring_at_seat": r_max.t_ring_at_seat_c})
        out[pos] = {"delta": [dmin, dmax], "rows": rows}
    # gap map (DE) for the figure
    if not quick:
        ring = ring_of(cfg, "DE")
        dmin, dmax = tolerance_interference(cfg["fits"]["shaft"][fit_of("DE")], bore)
        dts_map = list(range(40, 101, 5))
        ttrs = [0, 10, 20, 30, 45, 60, 90, 120]
        out["map_DE"] = {"dT": dts_map, "t_tr": ttrs,
                         "min_gap": [[simulate_hot_mount(ring, dmax, dT, t, 20.0, cfg).min_gap_um for t in ttrs] for dT in dts_map]}
    return out


def choose_limits(cfg, pw: dict, pwin: dict) -> tuple[dict, float]:
    """Recipe limits per bearing and the window-minimum heating dT for M3 (DOSSIER §6.3)."""
    tn = t_transfer_nominal(cfg)
    allowance = 15.0         # s of robot pause the recipe must tolerate without a re-heat [A]
    lim = {}
    dT_min = None
    for dT in sorted({r["dT"] for r in pw["DE"]["rows"]}):
        rows = [next(r for r in pw[p]["rows"] if r["dT"] == dT) for p in POS]
        ok = all(r["t_tr_max"] >= tn + allowance and r["min_gap_slow_5mm_s"] >= cfg["limits"]["min_gap_margin_um"]
                 for r in rows)
        if ok:
            dT_min = dT
            break
    for pos in POS:
        rows = {r["dT"]: r for r in pw[pos]["rows"]}
        lim[pos] = {"by_dT": {}}
        for dT, r in rows.items():
            hold = max(x for x in (r["t_full_grip_dmax"], r["t_full_grip_dmin"]) if x is not None)
            lim[pos]["by_dT"][dT] = {"t_tr_max": r["t_tr_max"], "hold_s": math.ceil(1.2 * hold)}
        lim[pos]["press_window"] = pwin[pos]["window"]
        lim[pos]["insert_max_s"] = 1.5 * (cfg["bearings"][pos]["B"] + cfg["mounting"]["seat_lead_mm"]) / 5.0
    dT_min = dT_min or 90.0
    # full-grip time against the actual interference at the window minimum (for a per-part hold, M5)
    bore = cfg["fits"]["bore_dmp"]["80_120"]
    for pos in POS:
        ring = ring_of(cfg, pos)
        dmin, dmax = tolerance_interference(cfg["fits"]["shaft"][fit_of(pos)], bore)
        ds = list(np.linspace(dmin, dmax, 8))
        tf = [simulate_hot_mount(ring, d, dT_min, tn, 20.0, cfg).t_full_grip_s for d in ds]
        lim[pos]["hold_vs_delta"] = {"delta": [float(x) for x in ds], "t_full": tf}
    return lim, dT_min


def press_window(cfg) -> dict:
    """F-x window at x1 = 0.4 B and x2 = 0.9 B from the fit tolerance and the friction range; what it can and cannot
    tell (DOSSIER §7)."""
    E = cfg["steel"]["E"]
    bore = cfg["fits"]["bore_dmp"]["80_120"]
    sm = smoothing_um(cfg["fits"])
    out = {}
    rng = np.random.default_rng(3)
    for pos in POS:
        ring = ring_of(cfg, pos)
        dmin, dmax = tolerance_interference(cfg["fits"]["shaft"][fit_of(pos)], bore)
        res = {}
        for label, mus in (("mu_range", cfg["mounting"]["mu"]), ("mu_controlled", cfg["mounting"]["mu_controlled"])):
            x1, x2 = 0.4 * ring.B, 0.9 * ring.B
            f = lambda x, d, m: float(press_force(x, max(d - sm, 0.0), ring, E, m)) / 1e3  # noqa: E731
            w = {"x1": x1, "x2": x2, "f1": [f(x1, dmin, mus[0]), f(x1, dmax, mus[1])],
                 "f2": [f(x2, dmin, mus[0]), f(x2, dmax, mus[1])]}
            # smallest interference the window rejects for sure (worst friction): F(x2, d, mu_hi) < f2_lo
            d_reject = sm + (dmin - sm) * mus[0] / mus[1]
            # a shaft from the wrong tolerance class (k5 instead of m5 on the DE seat, or a 5 um undersize grind)
            wrong = {}
            for name, shaft in (("k5_shaft", cfg["fits"]["shaft"]["k5_80_120"]), ("undersize_5um", [x - 5 for x in cfg["fits"]["shaft"][fit_of(pos)]])):
                s = cfg["fits"]["sigma_share"]
                sh = rng.normal(np.mean(shaft), s * (shaft[1] - shaft[0]), 20000).clip(*shaft)
                bo = rng.normal(np.mean(bore), s * (bore[1] - bore[0]), 20000).clip(*bore)
                mu = rng.uniform(mus[0], mus[1], 20000)
                ff = press_force(x2, np.maximum(sh - bo - sm, 0.0), ring, E, mu) / 1e3
                wrong[name] = float(np.mean((ff < w["f2"][0]) | (ff > w["f2"][1])))
            res[label] = {"window": w, "delta_rejected_below_um": d_reject, "wrong_part_rejected_share": wrong,
                          "ambiguity_factor": mus[1] / mus[0]}
        out[pos] = {"window": res["mu_range"]["window"], **res,
                    "friction_work_j": float(press_force(ring.B, dmax, ring, E, cfg["mounting"]["mu"][1])) * ring.B * 1e-3 / 2}
    return out


def closed_loop(plc, cfg, lim, dT_min, quick=False) -> tuple[dict, dict]:
    tr = {}
    out = {}
    bore = cfg["fits"]["bore_dmp"]["80_120"]
    for key, alt in cfg["alternatives"].items():
        a = dict(alt)
        if a["dT"] == "window":
            a["dT"] = float(dT_min)
        cfg_a = copy.deepcopy(cfg)
        cfg_a["alternatives"][key] = a
        out[key] = {"dT": a["dT"]}
        for pos in POS:
            l = {pos: dict(lim[pos]["by_dT"].get(int(a["dT"]), {}), press_window=lim[pos]["press_window"])}
            dmin, dmax = tolerance_interference(cfg["fits"]["shaft"][fit_of(pos)], bore)
            hv = lim[pos]["hold_vs_delta"]
            hold_of = (lambda d: math.ceil(1.2 * float(np.interp(d, hv["delta"], hv["t_full"])))) if a.get("adaptive") else (lambda d: None)  # noqa: E731
            rmax = run_mount(plc, cfg_a, key, pos, l, Scenario(delta_um=dmax, hold_s=hold_of(dmax)), keep_fx=True)
            rmin = run_mount(plc, cfg_a, key, pos, l, Scenario(delta_um=dmin, hold_s=hold_of(dmin)), keep_fx=True)
            out[key][pos] = {"dmax": {k: v for k, v in rmax.items() if k != "fx"}, "dmin": {k: v for k, v in rmin.items() if k != "fx"},
                             "recipe": l[pos] if a["method"] == "hot" else {"press_window": l[pos]["press_window"]}}
            if a["method"] == "press":
                tr[f"fx_{key}_{pos}_dmax"] = rmax["fx"]
                tr[f"fx_{key}_{pos}_dmin"] = rmin["fx"]
    # scenarios (DE bearing)
    sc = {}
    if not quick:
        a1 = cfg["alternatives"]["M2"]
        l = {"DE": dict(lim["DE"]["by_dT"][int(a1["dT"])], press_window=lim["DE"]["press_window"])}
        dmin, dmax = tolerance_interference(cfg["fits"]["shaft"][fit_of("DE")], bore)
        sc["stop_mid_slide_10s_90K"] = run_mount(plc, cfg, "M2", "DE", l, Scenario(delta_um=dmax, stop_mid_s=10.0))
        cfg3 = copy.deepcopy(cfg)
        cfg3["alternatives"]["M3"]["dT"] = float(dT_min)
        l3 = {"DE": dict(lim["DE"]["by_dT"][int(dT_min)], press_window=lim["DE"]["press_window"])}
        sc["slow_insert_5mm_s_window_min"] = run_mount(plc, cfg3, "M3", "DE", l3, Scenario(delta_um=dmax, v_insert=5.0))
        sc["stop_mid_slide_10s_window_min"] = run_mount(plc, cfg3, "M3", "DE", l3, Scenario(delta_um=dmax, stop_mid_s=10.0))
        cfg50 = copy.deepcopy(cfg)
        cfg50["alternatives"]["M3"]["dT"] = 50.0
        l50 = {"DE": dict(lim["DE"]["by_dT"][50], press_window=lim["DE"]["press_window"])}
        sc["robot_pause_90s_at_50K_with_interlock"] = run_mount(plc, cfg50, "M3", "DE", l50, Scenario(delta_um=dmax, transfer_delay_s=90.0))
        l50n = {"DE": dict(l50["DE"], t_tr_max=1e6)}
        sc["robot_pause_90s_at_50K_no_interlock"] = run_mount(plc, cfg50, "M3", "DE", l50n, Scenario(delta_um=dmax, transfer_delay_s=90.0))
        l_short = {"DE": dict(l["DE"], hold_s=5.0)}
        sc["hold_5s_instead_of_recipe"] = run_mount(plc, cfg, "M2", "DE", l_short, Scenario(delta_um=dmin))
        lp = {"DE": dict(press_window=lim["DE"]["press_window"])}
        k5 = cfg["fits"]["shaft"]["k5_80_120"]
        sc["press_k5_shaft_low"] = run_mount(plc, cfg, "M4", "DE", lp, Scenario(delta_um=k5[0] - bore[1] + 2, mu=0.15), keep_fx=True)
        sc["press_dry_seat_mu_0.30_dmax"] = run_mount(plc, cfg, "M4", "DE", lp, Scenario(delta_um=dmax, mu=0.30), keep_fx=True)
        sc["press_chip_1mm_short"] = run_mount(plc, cfg, "M4", "DE", lp, Scenario(delta_um=dmax, seat_blocked_mm=1.0), keep_fx=True)
        for k in ("press_k5_shaft_low", "press_dry_seat_mu_0.30_dmax", "press_chip_1mm_short"):
            tr[f"fx_{k}"] = sc[k].pop("fx")
    return out, sc, tr


def takt(cfg, cl: dict) -> dict:
    """Station cycle per rotor. One robot, one heater per bearing size (heating runs during the previous rotor),
    rotor exchange by the conveyor. With a hold-down clamp the robot leaves at once, but the rotor can only leave when
    the last clamp opens; the mounting order decides how much of the hold is hidden behind the robot's other work."""
    ln = cfg["line"]
    avail = ln["shift_hours"] * 3600 * ln["shifts"] * ln["oee"]
    rotors = ln["rotors_per_day_today"] * ln["multiplier"]
    takt_s = avail / rotors
    out = {"takt_s": takt_s, "rotors_per_day": rotors, "alts": {}}
    for key, a in cl.items():
        m = cfg["alternatives"][key]
        rob = {p: max(a[p]["dmax"]["robot_busy"], a[p]["dmin"]["robot_busy"]) for p in POS}
        heat = max(max(a[p]["dmax"]["t_heat"], a[p]["dmin"]["t_heat"]) for p in POS)
        orders = {}
        for order in (("DE", "NDE"), ("NDE", "DE")):
            first, last = order
            press = 0.0
            if m["method"] == "press":
                press = sum(cfg["press"]["approach_s"] + cfg["bearings"][p]["B"] / cfg["press"]["v_mm_s"] + 2.0 for p in POS)
            wait = 0.0
            if m["hold"] == "clamp":
                after_first = rob[last]                  # robot work after the first clamp closed
                after_last = 3.0                          # retract clear of the rotor [A]
                if m.get("adaptive"):
                    hs = {p: hold_samples(cfg, a, p) for p in POS}
                    wait = float(np.mean(np.maximum(0.0, np.maximum(hs[first] - after_first - after_last, hs[last] - after_last))))
                else:
                    hold = {p: a[p]["recipe"].get("hold_s", 0.0) for p in POS}
                    wait = max(0.0, hold[first] - after_first - after_last, hold[last] - after_last)
            station = rob[first] + rob[last] + press + wait + ln["rotor_handling_s"]
            orders["-".join(order)] = {"station_s": station, "clamp_wait_s": wait, "press_s": press}
        best = min(orders, key=lambda k: orders[k]["station_s"])
        cycle = max(orders[best]["station_s"], heat)
        out["alts"][key] = {"robot_s": sum(rob.values()), "heater_s": heat, "orders": orders, "best_order": best,
                            "clamp_wait_s": orders[best]["clamp_wait_s"], "press_s": orders[best]["press_s"],
                            "cycle_s": cycle, "meets_takt": cycle <= takt_s, "rotors_per_day": avail / cycle}
    return out


LIM_CACHE: dict = {}


def hold_samples(cfg, a, pos, n=4000, seed=5):
    """Per-part hold times [s] for gauged interference: 1.2 x full-grip time at the part's own interference."""
    hv = LIM_CACHE[pos]["hold_vs_delta"]
    bore = cfg["fits"]["bore_dmp"]["80_120"]
    shaft = cfg["fits"]["shaft"][fit_of(pos)]
    s = cfg["fits"]["sigma_share"]
    rng = np.random.default_rng(seed + (pos == "NDE"))
    d = rng.normal(np.mean(shaft), s * (shaft[1] - shaft[0]), n).clip(*shaft) - rng.normal(np.mean(bore), s * (bore[1] - bore[0]), n).clip(*bore)
    return np.ceil(1.2 * np.interp(d, hv["delta"], hv["t_full"]))


def gates(cfg, cl, sc, tk, pw, pwin, dT_min) -> dict:
    g = {}
    for key, a in cl.items():
        m = cfg["alternatives"][key]["method"]
        ok_seat = all(a[p][c]["verdict"] == 1 for p in POS for c in ("dmax", "dmin"))
        if m == "hot":
            ok_seat = ok_seat and all(a[p][c].get("seat_held_until_full_grip", False) for p in POS for c in ("dmax", "dmin"))
            dT = a["dT"]
            margin = min(next(r for r in pw[p]["rows"] if r["dT"] == int(dT))["min_gap_nominal"] for p in POS)
            no_seize = margin >= cfg["limits"]["min_gap_margin_um"]
            temp_ok = cfg["mounting"]["t_shaft"] + dT <= cfg["mounting"]["t_bearing_max"]
        else:
            no_seize = True
            temp_ok = True
        g[key] = {"SR-01 seat": ok_seat, "SR-02 no seizure": no_seize, "SR-03 120 °C": temp_ok,
                  "SR-05 takt": tk["alts"][key]["meets_takt"]}
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    t0 = time.time()
    cfg = load_config()
    plc = Plc()
    fs = fits_study(cfg)
    pw = process_window(cfg, quick=a.quick)
    pwin = press_window(cfg)
    lim, dT_min = choose_limits(cfg, pw, pwin)
    LIM_CACHE.update(lim)
    cl, sc, tr = closed_loop(plc, cfg, lim, dT_min, quick=a.quick)
    tk = takt(cfg, cl)
    out = {"t_transfer_nominal_s": t_transfer_nominal(cfg), "fits": fs, "process_window": pw, "press": pwin,
           "limits": lim, "dT_window_min": dT_min, "closed_loop": cl, "scenarios": sc, "takt": tk,
           "gates": gates(cfg, cl, sc, tk, pw, pwin, dT_min)}
    if not a.quick:
        ring = ring_of(cfg, "DE")
        dmin, dmax = tolerance_interference(cfg["fits"]["shaft"][fit_of("DE")], cfg["fits"]["bore_dmp"]["80_120"])
        for dT in (90, int(dT_min)):
            r = simulate_hot_mount(ring, dmax, dT, t_transfer_nominal(cfg), 20.0, cfg, keep_trace=True)
            tr[f"slide_DE_{dT}"] = r.trace
    out["runtime_s"] = time.time() - t0
    name = "studies_quick.json" if a.quick else "studies.json"
    (ROOT / "results" / name).write_text(json.dumps(out, indent=1, default=float))
    if not a.quick:
        (ROOT / "results" / "traces.json").write_text(json.dumps(tr, default=float))
    print(f"runtime {out['runtime_s']:.0f} s; window minimum dT {dT_min}; takt {tk['takt_s']:.0f} s")
    for k, v in tk["alts"].items():
        print(k, f"cycle {v['cycle_s']:.0f} s (robot {v['robot_s']:.0f}, heater {v['heater_s']:.0f}, clamp wait {v['clamp_wait_s']:.0f}, order {v['best_order']})", out["gates"][k])


if __name__ == "__main__":
    main()
