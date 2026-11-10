"""The robot program, the I/O map and the ST controller agree; the compiled controller behaves as specified."""
import re
from pathlib import Path

import pytest
import yaml

from mount import plc as plcmod
from mount.plant import Scenario, load_config, run_mount

ROOT = Path(__file__).resolve().parents[1]
IO = yaml.safe_load((ROOT / "robot" / "io_map.yaml").read_text())
LS = (ROOT / "robot" / "BRG_MOUNT.ls").read_text()
ST = (ROOT / "plc" / "st" / "mount_cell.st").read_text()


def st_names(section):
    body = re.search(rf"\b{section}\b(.*?)\bEND_VAR\b", ST, flags=re.S).group(1)
    body = re.sub(r"\(\*.*?\*\)", "", body, flags=re.S)
    return {d.split(":")[0].strip() for d in body.split(";") if ":" in d}


def test_every_robot_signal_in_the_program_is_in_the_io_map_with_the_same_name():
    used = set(re.findall(r"\b(DI|DO)\[(\d+):([A-Z_ ]+)\]", LS))
    known = {("DI", str(k), v["name"]) for k, v in IO["robot_di"].items()} | \
            {("DO", str(k), v["name"]) for k, v in IO["robot_do"].items()} | \
            {("DI", str(k), v["name"]) for k, v in IO["robot_sensor_di"].items()}
    assert used and used <= known, used - known


def test_io_map_points_at_real_plc_variables():
    ins, outs = st_names("VAR_INPUT"), st_names("VAR_OUTPUT")
    assert {v["plc"] for v in IO["robot_do"].values()} <= ins
    assert {v["plc"] for v in IO["robot_di"].values()} <= outs


def test_robot_program_header_line_count_matches():
    n = int(re.search(r"LINE_COUNT\s*=\s*(\d+);", LS).group(1))
    body = LS[LS.index("/MN"):LS.index("/POS")]
    assert n == len(re.findall(r"^\s*\d+:", body, flags=re.M))


needs_lib = pytest.mark.skipif(not plcmod.available(), reason="build/plc/libplc.so not built")
CFG = load_config()


def limits(dT=90, hold=60.0, ttr=300.0):
    w = {"x1": 20.0, "x2": 45.0, "f1": [2.77, 24.2], "f2": [6.24, 54.5]}
    return {p: {"t_tr_max": ttr, "hold_s": hold, "press_window": w} for p in ("DE", "NDE")}


@pytest.fixture(scope="module")
def plc():
    return plcmod.Plc()


@needs_lib
def test_hot_mount_completes_and_holds_for_the_recipe_time(plc):
    r = run_mount(plc, CFG, "M2", "DE", limits(hold=60.0), Scenario(delta_um=30.0))
    assert r["verdict"] == 1 and r["t_hold_plc"] == pytest.approx(60.0, abs=0.06) and not r["seized"]


@needs_lib
def test_slow_transfer_returns_the_bearing_for_a_reheat(plc):
    r = run_mount(plc, CFG, "M2", "DE", limits(ttr=5.0), Scenario(delta_um=30.0, transfer_delay_s=20.0))
    assert r["reheats"] == 1 and r["verdict"] == 1


@needs_lib
def test_seized_ring_is_reported_not_seated(plc):
    cfg = {**CFG, "alternatives": {**CFG["alternatives"], "MX": {"name": "x", "method": "hot", "dT": 35.0, "hold": "clamp"}}}
    r = run_mount(plc, cfg, "MX", "DE", limits(), Scenario(delta_um=48.0))
    assert r["seized"] and r["verdict"] == 2 and r["nok"] == plc.constants["N_NOT_SEATED"]


@needs_lib
def test_press_inside_the_window_passes_and_an_obstacle_fails(plc):
    ok = run_mount(plc, CFG, "M4", "DE", limits(), Scenario(delta_um=30.0, mu=0.15))
    bad = run_mount(plc, CFG, "M4", "DE", limits(), Scenario(delta_um=30.0, mu=0.15, seat_blocked_mm=1.0))
    assert ok["verdict"] == 1 and bad["verdict"] == 2 and bad["nok"] == plc.constants["N_NO_SEAT"]


@needs_lib
def test_safety_stop_faults_with_every_command_off_in_the_same_scan(plc):
    plc.reset()
    plc.set(rcMethod=1, rcDeltaT=90.0, rcTMax=120.0, cycleTime=0.05, safetyOk=True, rbReady=True, mTShaft=20.0,
            mTBearing=20.0, cmdStart=True)
    plc.scan()
    assert plc.get("QHEATERON")
    plc.set(safetyOk=False)
    plc.scan()
    assert plc.get("STATE") == plc.constants["S_FAULT"]
    assert not any(plc.get(n) for n in ("QHEATERON", "QRBPICK", "QRBINSERT", "QCLAMP", "QPRESSGO", "QRBABORT"))


@needs_lib
def test_bearing_overtemperature_faults(plc):
    plc.reset()
    plc.set(rcMethod=1, rcDeltaT=200.0, rcTMax=120.0, cycleTime=0.05, safetyOk=True, rbReady=True, mTShaft=20.0,
            mTBearing=125.0, cmdStart=True)
    plc.scan()
    plc.scan()
    assert plc.get("STATE") == plc.constants["S_FAULT"] and plc.get("NOKCODE") == plc.constants["N_OVERTEMP"]
