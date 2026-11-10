"""Write the result tables into DOSSIER.md and README.md between <!-- TABLE:name --> ... <!-- /TABLE --> markers.
Every number comes from results/studies.json; none is typed by hand."""
import json
import re
from pathlib import Path

from mount.plant import load_config

ROOT = Path(__file__).resolve().parents[1]
POS = ("DE", "NDE")


def f(x, nd=1):
    return "—" if x is None else f"{x:.{nd}f}"


def t_fits(d):
    rows = ["| Bearing | Shaft | Clearance | Interference (µm) | Max pressure (MPa) | Max hoop stress (MPa) | Cold press force, µ 0.20 (kN) | "
            "Operating clearance mean / worst case (µm) | No clearance in operation |", "|---|---|---|---|---|---|---|---|---|"]
    for pos in POS:
        for o in d["fits"][pos]["options"]:
            rows.append(f"| {pos} | {o['shaft']} | {o['clearance']} | {o['delta_min']:g}–{o['delta_max']:g} | {f(o['p_max_mpa'])} | "
                        f"{f(o['hoop_max_mpa'], 0)} | {f(o['f_press_max_kn'], 0)} | {f(o['mean'])} / {f(o['worst_case'])} | "
                        f"{100 * o['p_negative']:.3f} % |")
    return "\n".join(rows)


def t_dt(d):
    rows = ["| Inner ring warmer than outer (K) | " + " | ".join(f"{r['dt_k']:g}" for r in d["fits"]["DE"]["dt_sweep"]) + " |",
            "|---|" + "---|" * len(d["fits"]["DE"]["dt_sweep"])]
    for pos, lab in (("DE", "DE m5 / C3"), ("NDE", "NDE k5 / C3")):
        rows.append(f"| {lab}: no clearance | " + " | ".join(f"{100 * r['p_negative']:.3f} %" for r in d["fits"][pos]["dt_sweep"]) + " |")
    return "\n".join(rows)


def t_window(d):
    rows = ["| Bearing | ΔT (K) | Heating (s) | Smallest gap 20 / 5 mm/s (µm) | Longest heater-to-seat time (s) | Longest stop half way (s) | "
            "Full grip, tightest / loosest fit (s) | Face gap if released at first grip, tightest / loosest (µm) |", "|---|---|---|---|---|---|---|---|"]
    for pos in POS:
        for r in d["process_window"][pos]["rows"]:
            ttr = "≥ 300" if r["t_tr_max"] > 299 else f(r["t_tr_max"], 0)
            rows.append(f"| {pos} | {r['dT']} | {f(r['t_heat_s'], 0)} | {f(r['min_gap_nominal'])} / {f(r['min_gap_slow_5mm_s'])} | {ttr} | "
                        f"{f(r['t_stop_mid_max'])} | {f(r['t_full_grip_dmax'])} / {f(r['t_full_grip_dmin'])} | "
                        f"{f(r['face_gap_first_grip_dmax'])} / {f(r['face_gap_first_grip_dmin'])} |")
    return "\n".join(rows)


def t_press(d):
    rows = ["| Bearing | Friction range | Window at x1 (kN) | Window at x2 (kN) | Interference surely rejected below (µm) | "
            "k5 shafts rejected | 5 µm undersize grind rejected | Friction work at max fit (J) |", "|---|---|---|---|---|---|---|---|"]
    for pos in POS:
        p = d["press"][pos]
        for lab, key in (("µ 0.10–0.20", "mu_range"), ("µ 0.12–0.15 (paste)", "mu_controlled")):
            w = p[key]["window"]
            wr = p[key]["wrong_part_rejected_share"]
            rows.append(f"| {pos} | {lab} | {f(w['f1'][0])}–{f(w['f1'][1])} | {f(w['f2'][0])}–{f(w['f2'][1])} | "
                        f"{f(p[key]['delta_rejected_below_um'])} | {100 * wr['k5_shaft']:.1f} % | {100 * wr['undersize_5um']:.2f} % | "
                        f"{f(p['friction_work_j'], 0)} |")
    return "\n".join(rows)


def t_takt(d, cfg):
    tk = d["takt"]
    rows = ["| | Alternative | ΔT (K) | Robot (s) | Press (s) | Wait for clamp (s) | Heater (s) | Best order | Cycle per rotor (s) | "
            f"Takt {tk['takt_s']:.0f} s |", "|---|---|---|---|---|---|---|---|---|---|"]
    for k, a in tk["alts"].items():
        rows.append(f"| {k} | {cfg['alternatives'][k]['name']} | {f(d['closed_loop'][k]['dT'], 0)} | {f(a['robot_s'], 0)} | "
                    f"{f(a['press_s'], 0)} | {f(a['clamp_wait_s'], 0)} | {f(a['heater_s'], 0)} | {a['best_order']} | "
                    f"{f(a['cycle_s'], 0)} | {'meets' if a['meets_takt'] else '**misses**'} |")
    return "\n".join(rows)


def t_gates(d):
    alts = list(d["gates"])
    names = list(next(iter(d["gates"].values())))
    rows = ["| Requirement | " + " | ".join(alts) + " |", "|---|" + "---|" * len(alts)]
    for n in names:
        rows.append(f"| {n} | " + " | ".join("pass" if d["gates"][a][n] else "**fail**" for a in alts) + " |")
    return "\n".join(rows)


def t_scen(d):
    nok = {0: "—", 1: "transfer too slow", 2: "not seated", 3: "over-temperature", 4: "force at x1", 5: "force at x2",
           6: "no seat", 7: "over-force", 8: "re-heats used up", 9: "safety"}
    label = {
        "stop_mid_slide_10s_90K": "Protective stop of 10 s half way, 90 K (M2)",
        "slow_insert_5mm_s_window_min": "Slide-on at 5 mm/s, window minimum (M3)",
        "stop_mid_slide_10s_window_min": "Protective stop of 10 s half way, window minimum (M3)",
        "robot_pause_90s_at_50K_with_interlock": "Robot paused 90 s in transfer, 50 K, transfer interlock on",
        "robot_pause_90s_at_50K_no_interlock": "Same, interlock off",
        "hold_5s_instead_of_recipe": "Hold 5 s instead of the recipe, loosest fit (M2)",
        "press_k5_shaft_low": "Press: k5 shaft at 5 µm, µ 0.10 (M4)",
        "press_dry_seat_mu_0.30_dmax": "Press: dry seat µ 0.30, 48 µm (M4)",
        "press_chip_1mm_short": "Press: chip, stroke stops 1 mm short (M4)"}
    rows = ["| Scenario (DE bearing) | PLC verdict | Reason | Re-heats | Smallest gap (µm) | Held until full grip |", "|---|---|---|---|---|---|"]
    for k, v in d["scenarios"].items():
        held = v.get("seat_held_until_full_grip")
        rows.append(f"| {label.get(k, k)} | {'OK' if v['verdict'] == 1 else 'NOK'} | {nok.get(v['nok'], v['nok'])} | {v['reheats']} | "
                    f"{f(v.get('min_gap_um'))} | {'—' if held is None else ('yes' if held else '**no**')} |")
    return "\n".join(rows)


def t_hold(d):
    rows = ["| Interference (µm) | " + " | ".join(f"{x:.0f}" for x in d["limits"]["DE"]["hold_vs_delta"]["delta"]) + " |",
            "|---|" + "---|" * 8]
    for pos in POS:
        h = d["limits"][pos]["hold_vs_delta"]
        if pos == "NDE":
            rows.append("| NDE interference (µm) | " + " | ".join(f"{x:.0f}" for x in h["delta"]) + " |")
        rows.append(f"| {pos} full grip (s) | " + " | ".join(f(x) for x in h["t_full"]) + " |")
    return "\n".join(rows)


def fill(path: Path, tables: dict):
    txt = path.read_text(encoding="utf-8")
    for name, body in tables.items():
        txt = re.sub(rf"(<!-- TABLE:{name} -->)(.*?)(<!-- /TABLE -->)", lambda m: f"{m.group(1)}\n{body}\n{m.group(3)}", txt, flags=re.S)
    path.write_text(txt, encoding="utf-8")


if __name__ == "__main__":
    cfg = load_config()
    d = json.loads((ROOT / "results" / "studies.json").read_text())
    tables = {"fits": t_fits(d), "dt": t_dt(d), "window": t_window(d), "press": t_press(d), "takt": t_takt(d, cfg),
              "gates": t_gates(d), "scen": t_scen(d), "hold": t_hold(d)}
    for doc in ("DOSSIER.md", "README.md"):
        if (ROOT / doc).exists():
            fill(ROOT / doc, tables)
    print("tables:", ", ".join(tables))
