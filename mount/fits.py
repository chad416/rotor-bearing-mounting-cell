"""Interference fit of a bearing inner ring on a solid shaft: tolerance stack, Lame pressure, press force,
raceway expansion and operating clearance.

Lame (thick-walled cylinders, same E and nu for shaft and ring, solid shaft):
  diametral interference delta = d * p / E * [ (b^2 + a^2)/(b^2 - a^2) + nu ] + d * p / E * (1 - nu)
                               = d * p / E * 2 b^2 / (b^2 - a^2)
  so  p = E * delta * (b^2 - a^2) / (2 d b^2)          a = d/2 (bore radius), b = d1/2 (ring outer radius)
  ring outer-surface displacement under internal pressure p:  u(b) = 2 p a^2 b / (E (b^2 - a^2))
  -> raceway diameter growth  dd1 = 2 u(b),  ratio f1 = dd1 / delta  (the ratio a catalogue reads from a diagram)
Press-in force at engagement length x:  F = mu * p * pi * d * x.
Clearance loss in operation from the inner/outer ring temperature difference: 0.012 * dT * dm um (dm in mm) [B1].
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Ring:
    d: float     # bore, mm
    D: float     # outside diameter, mm
    B: float     # width, mm
    d1: float    # inner-ring outer diameter, mm
    mass: float  # whole bearing, kg

    @classmethod
    def from_config(cls, b: dict, d1_share: float) -> "Ring":
        return cls(d=b["d"], D=b["D"], B=b["B"], d1=b["d"] + d1_share * (b["D"] - b["d"]), mass=b["mass"])

    def inner_ring_mass(self, rho: float) -> float:
        return rho * math.pi / 4 * ((self.d1 * 1e-3) ** 2 - (self.d * 1e-3) ** 2) * self.B * 1e-3

    @property
    def dm(self) -> float:
        return 0.5 * (self.d + self.D)


def lame_pressure(delta_um, ring: Ring, E: float) -> np.ndarray:
    """Contact pressure [Pa] for a diametral interference delta [um] (negative delta -> 0)."""
    a, b = ring.d / 2, ring.d1 / 2
    d = np.maximum(np.asarray(delta_um, float), 0.0) * 1e-6
    return E * d * (b * b - a * a) / (2 * (ring.d * 1e-3) * b * b)


def raceway_ratio(ring: Ring) -> float:
    """f1 = raceway diameter growth / interference (independent of E and delta)."""
    a, b = ring.d / 2, ring.d1 / 2
    # dd1/delta = [4 p a^2 b / (E (b^2-a^2))] / [2 d p b^2 / (E (b^2-a^2))] = 4 a^2 b / (2 d b^2) = a / b
    return a / b


def press_force(x_mm, delta_um, ring: Ring, E: float, mu: float) -> np.ndarray:
    """Press-in force [N] after x mm of engagement."""
    p = lame_pressure(delta_um, ring, E)
    x = np.clip(np.asarray(x_mm, float), 0.0, ring.B) * 1e-3
    return mu * p * math.pi * ring.d * 1e-3 * x


def hoop_stress_bore(delta_um, ring: Ring, E: float) -> np.ndarray:
    """Tangential stress at the ring bore [Pa]: p (b^2 + a^2)/(b^2 - a^2)."""
    a, b = ring.d / 2, ring.d1 / 2
    return lame_pressure(delta_um, ring, E) * (b * b + a * a) / (b * b - a * a)


def tolerance_interference(shaft: list[float], bore: list[float]) -> tuple[float, float]:
    """Min and max diametral interference [um] from shaft and bore limit deviations [um]."""
    return shaft[0] - bore[1], shaft[1] - bore[0]


def smoothing_um(fc: dict) -> float:
    return fc["smoothing_k"] * (fc["rz_shaft_um"] + fc["rz_bore_um"])


def operating_clearance_mc(ring: Ring, shaft: list[float], bore: list[float], clearance: list[float], fc: dict,
                           n: int = 200_000, seed: int = 1, dt_k: float | None = None) -> dict:
    """Monte Carlo of the operating radial clearance [um]: catalogue clearance (uniform in its class), minus the fit
    (raceway growth f1 x effective interference), minus the inner/outer temperature difference."""
    rng = np.random.default_rng(seed)
    s = fc["sigma_share"]
    sh = rng.normal((shaft[0] + shaft[1]) / 2, s * (shaft[1] - shaft[0]), n).clip(shaft[0], shaft[1])
    bo = rng.normal((bore[0] + bore[1]) / 2, s * (bore[1] - bore[0]), n).clip(bore[0], bore[1])
    delta = sh - bo
    d_eff = np.maximum(delta - smoothing_um(fc), 0.0)
    c0 = rng.uniform(clearance[0], clearance[1], n)
    dt = fc["dt_inner_outer_k"] if dt_k is None else dt_k
    c_op = c0 - raceway_ratio(ring) * d_eff - 0.012 * dt * ring.dm
    wc = clearance[0] - raceway_ratio(ring) * max(tolerance_interference(shaft, bore)[1] - smoothing_um(fc), 0.0) - 0.012 * dt * ring.dm
    return {"p_negative": float(np.mean(c_op < 0)), "mean": float(c_op.mean()), "p01": float(np.percentile(c_op, 0.1)),
            "worst_case": float(wc), "delta_mean": float(delta.mean()), "delta_p999": float(np.percentile(delta, 99.9)),
            "delta_p001": float(np.percentile(delta, 0.1))}
