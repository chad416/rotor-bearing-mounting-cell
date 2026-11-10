"""All dossier figures (PNG, docs/img) from results/studies.json and results/traces.json."""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
IMG.mkdir(parents=True, exist_ok=True)
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
LIMIT = "#e34948"
SEQ = LinearSegmentedColormap.from_list("seq", ["#fdeee7", "#f3b393", "#eb6834", "#eef4fc", "#9cc1ee", "#2a78d6", "#0d3a73"])
plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.6,
                     "figure.facecolor": SURF, "axes.facecolor": SURF, "legend.frameon": False})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(IMG / name, dpi=150)
    plt.close(fig)


def fig_window(d):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.6, 3.8))
    for pos, col in (("DE", C[0]), ("NDE", C[1])):
        rows = d["process_window"][pos]["rows"]
        dT = [r["dT"] for r in rows]
        a1.plot(dT, [r["min_gap_nominal"] for r in rows], marker="o", ms=4, color=col, label=f"{pos}, slide-on 20 mm/s")
        a1.plot(dT, [r["min_gap_slow_5mm_s"] for r in rows], marker="o", ms=4, color=col, ls="--", label=f"{pos}, 5 mm/s")
        a2.plot(dT, [r["t_full_grip_dmin"] for r in rows], marker="o", ms=4, color=col, label=f"{pos}, loosest fit")
        a2.plot(dT, [r["t_full_grip_dmax"] if r["t_full_grip_dmax"] else np.nan for r in rows], marker="o", ms=4,
                color=col, ls="--", label=f"{pos}, tightest fit")
    a1.axhline(5, color=LIMIT, ls="--", lw=1)
    a1.text(41, 6.5, "SR-02 margin 5 µm", fontsize=7, color=INK2)
    a1.axhline(0, color=INK, lw=0.8)
    a1.axvline(d["dT_window_min"], color=INK2, ls=":", lw=1)
    a1.set_xlabel("heating above shaft temperature ΔT (K)")
    a1.set_ylabel("smallest radial gap while sliding (µm)")
    a1.set_title("Slide-on margin: set by the tightest fit", fontsize=9, loc="left")
    a1.legend(fontsize=7)
    a2.axvline(d["dT_window_min"], color=INK2, ls=":", lw=1)
    a2.set_xlabel("heating above shaft temperature ΔT (K)")
    a2.set_ylabel("time to grip over the full width (s)")
    a2.set_title("Hold time: set by the loosest fit, grows with ΔT", fontsize=9, loc="left")
    a2.legend(fontsize=7)
    save(fig, "process_window.png")


def fig_map(d):
    m = d["process_window"]["map_DE"]
    g = np.array(m["min_gap"])
    fig, ax = plt.subplots(figsize=(6.6, 3.8))
    im = ax.pcolormesh(np.arange(len(m["t_tr"]) + 1) - 0.5, np.array(m["dT"] + [m["dT"][-1] + 5]) - 2.5, g,
                       cmap=SEQ, vmin=-20, vmax=45, shading="flat")
    for i, dT in enumerate(m["dT"]):
        for j in range(len(m["t_tr"])):
            v = g[i, j]
            ax.text(j, dT, "seize" if v <= 0 else f"{v:.0f}", ha="center", va="center", fontsize=6.5,
                    color=INK if -5 < v < 30 else SURF)
    ax.set_xticks(range(len(m["t_tr"])))
    ax.set_xticklabels([str(t) for t in m["t_tr"]])
    ax.set_xlabel("time from heater to seat start (s)")
    ax.set_ylabel("ΔT (K)")
    ax.grid(False)
    cb = fig.colorbar(im, ax=ax)
    cb.set_label("smallest radial gap (µm)")
    ax.set_title("DE bearing, tightest fit (48 µm), slide-on 20 mm/s", fontsize=9, loc="left")
    save(fig, "gap_map.png")


def fig_slide(tr, d):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.6, 3.5))
    for k, col in ((f"slide_DE_{int(d['dT_window_min'])}", C[1]), ("slide_DE_90", C[0])):
        t = tr[k]
        lab = k.split("_")[-1] + " K"
        a1.plot(t["t"], t["gap_min"], color=col, label=lab)
        a2.plot(t["t"], t["t_ring"], color=col, label=f"{lab}: inner ring")
        a2.plot(t["t"], t["t_surf"], color=col, ls="--", lw=1.0, label=f"{lab}: shaft surface under it")
    a1.axhline(0, color=INK, lw=0.8)
    a1.set_xlabel("time from the start of the slide-on (s)")
    a1.set_ylabel("smallest gap under the ring (µm)")
    a1.set_title("Gap closes: grip (negative = interference building up)", fontsize=9, loc="left")
    a1.set_xlim(0, 60)
    a1.legend(fontsize=7)
    a2.set_xlabel("time from the start of the slide-on (s)")
    a2.set_ylabel("°C")
    a2.set_xlim(0, 60)
    a2.set_title("DE ring on the tightest fit", fontsize=9, loc="left")
    a2.legend(fontsize=7)
    save(fig, "slide_on.png")


def fig_fx(tr, d):
    w = d["press"]["DE"]["window"]
    fig, ax = plt.subplots(figsize=(7.6, 3.8))
    for k, col, lab in (("fx_M4_DE_dmax", C[0], "48 µm (tolerance max), µ 0.15"), ("fx_M4_DE_dmin", C[2], "13 µm (tolerance min), µ 0.15"),
                        ("fx_press_k5_shaft_low", C[3], "k5 shaft, 5 µm, µ 0.10"), ("fx_press_chip_1mm_short", C[6], "chip: stops 1 mm short"),
                        ("fx_press_dry_seat_mu_0.30_dmax", C[1], "dry seat, µ 0.30")):
        t = tr[k]
        ax.plot(t["x"], t["f"], color=col, label=lab, lw=1.4, ls="--" if "chip" in k else "-")
    for x, f in ((w["x1"], w["f1"]), (w["x2"], w["f2"])):
        ax.add_patch(Rectangle((x - 0.6, f[0]), 1.2, f[1] - f[0], fill=False, ec=INK, lw=1.2))
        ax.text(x + 0.9, f[1], f"window\n{f[0]:.1f}–{f[1]:.1f} kN", fontsize=7, va="top")
    ax.axhline(100, color=LIMIT, ls="--", lw=1)
    ax.text(30, 102, "press rating 100 kN", fontsize=7, color=INK2)
    ax.set_xlabel("engagement (mm); seat at 50 mm")
    ax.set_ylabel("press force (kN)")
    ax.set_xlim(0, 52)
    ax.set_ylim(0, 125)
    ax.set_title("Cold press of the DE bearing: force-displacement and window (µ 0.10–0.20)", fontsize=9, loc="left")
    ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(0.0, 0.93))
    save(fig, "press_fx.png")


def fig_clearance(d):
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for pos, col, lab in (("DE", C[0], "DE 6322, m5 / C3"), ("NDE", C[1], "NDE 6319, k5 / C3")):
        s = d["fits"][pos]["dt_sweep"]
        ax.semilogy([r["dt_k"] for r in s], [max(r["p_negative"], 1e-6) * 100 for r in s], marker="o", ms=4, color=col, label=lab)
    ax.axhline(0.1, color=LIMIT, ls="--", lw=1)
    ax.text(0.2, 0.13, "SR-04: 0.1 %", fontsize=7, color=INK2)
    ax.set_xlabel("inner ring warmer than outer ring in operation (K)")
    ax.set_ylabel("assemblies with no operating clearance (%)")
    ax.set_title("Operating clearance after the fit (Monte Carlo)", fontsize=9, loc="left")
    ax.legend(fontsize=7)
    save(fig, "clearance.png")


def fig_takt(d):
    tk = d["takt"]
    alts = list(tk["alts"])
    fig, ax = plt.subplots(figsize=(7.6, 3.6))
    x = np.arange(len(alts))
    rob = [tk["alts"][a]["robot_s"] for a in alts]
    wait = [tk["alts"][a]["clamp_wait_s"] for a in alts]
    prs = [tk["alts"][a]["press_s"] for a in alts]
    hand = [40.0] * len(alts)
    ax.bar(x - 0.18, rob, 0.34, color=C[0], label="robot (incl. holding)")
    ax.bar(x - 0.18, prs, 0.34, bottom=rob, color=C[6], label="press strokes")
    ax.bar(x - 0.18, wait, 0.34, bottom=np.add(rob, prs), color=C[3], label="waiting for the last clamp")
    ax.bar(x - 0.18, hand, 0.34, bottom=np.add(np.add(rob, prs), wait), color=C[2], label="rotor exchange")
    ax.bar(x + 0.18, [tk["alts"][a]["heater_s"] for a in alts], 0.34, color=C[1], label="heater (parallel)")
    ax.axhline(tk["takt_s"], color=LIMIT, ls="--", lw=1)
    ax.text(len(alts) - 0.5, tk["takt_s"] + 6, f"takt at 3× output {tk['takt_s']:.0f} s", fontsize=7, ha="right", color=INK2)
    for xi, a in zip(x, alts):
        ax.text(xi - 0.18, tk["alts"][a]["cycle_s"] + 6, f"{tk['alts'][a]['cycle_s']:.0f}", ha="center", fontsize=7)
    ax.set_xticks(x)
    ax.set_xticklabels(alts)
    ax.set_ylabel("s per rotor (2 bearings)")
    ax.set_title("Station cycle per rotor against the takt", fontsize=9, loc="left")
    ax.legend(fontsize=7, loc="upper right")
    save(fig, "takt.png")


def fig_hold(d):
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for pos, col in (("DE", C[0]), ("NDE", C[1])):
        h = d["limits"][pos]["hold_vs_delta"]
        ax.plot(h["delta"], h["t_full"], marker="o", ms=4, color=col, label=f"{pos}")
    ax.set_xlabel("measured diametral interference of the part (µm)")
    ax.set_ylabel("time to full grip (s)")
    ax.set_title(f"Hold time per part at ΔT = {d['dT_window_min']:.0f} K (basis of M5)", fontsize=9, loc="left")
    ax.legend(fontsize=7)
    save(fig, "hold_vs_interference.png")


def main():
    d = json.loads((ROOT / "results" / "studies.json").read_text())
    tr = json.loads((ROOT / "results" / "traces.json").read_text())
    fig_window(d)
    fig_map(d)
    fig_slide(tr, d)
    fig_fx(tr, d)
    fig_clearance(d)
    fig_takt(d)
    fig_hold(d)
    print("figures in", IMG)


if __name__ == "__main__":
    main()
