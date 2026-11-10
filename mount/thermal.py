"""Hot mounting of a bearing on a shaft: transfer, slide-on and lock, as a coupled transient.

Bodies
  inner ring   n axial nodes over its width B, radially lumped (Biot = h t / k < 0.02 even in contact), conducting
               axially; the rest of the bearing (balls, cage, outer ring) is one node coupled through the balls.
  shaft        solid steel cylinder in (r, z) finite volumes under and around the seat, explicit time stepping.
Coupling
  while a ring node is over the seat, it exchanges heat with the shaft surface cell under it through the radial
  gap g: h = k_air / (g + g_jump) + h_radiation, and h_contact once g <= 0 [A].
  g = (d_bore(T_ring) - d_shaft(T_shaft)) / 2, with
     d_bore  = d - delta + alpha d (T_ring - T0)                  (thin ring follows its own temperature)
     d_shaft = d + alpha d (Tbar_shaft(z) - T0)                   (slice-wise area-mean shaft temperature [D])
  The surface displacement of a solid cylinder with a radial temperature profile T(r) and free ends is
  u(R) = alpha R Tbar (area mean) in generalised plane strain; it is applied slice by slice (axial coupling of the
  displacement is neglected, an approximation stated in the dossier).
Kinematics
  t < 0: carried through air for t_transfer (convection + radiation); then the leading face slides from the seat lead
  to the shoulder at speed v. A ring node that closes its gap before the face reaches the shoulder seizes the ring.
  After seating the ring is held against the shoulder; the time of first and of full grip is recorded, and the
  axial gap that opens at the face if the ring were released at first grip: the part between the face and the grip
  point shrinks towards the grip point.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from mount.fits import Ring

SIGMA = 5.670374419e-8


@dataclass
class HotMountResult:
    seized: bool
    seize_pos_mm: float | None
    min_gap_um: float               # smallest radial gap over the seat while sliding
    gap_at_seat_start_um: float
    t_first_grip_s: float | None    # after seating
    grip_node_from_face: int | None
    t_full_grip_s: float | None
    face_gap_if_released_um: float | None
    t_ring_at_seat_c: float
    trace: dict


def simulate_hot_mount(ring: Ring, delta_um: float, dT: float, t_transfer: float, v_mm_s: float, cfg: dict,
                       t_after: float = 300.0, dt: float = 0.05, n_ring: int = 20, keep_trace: bool = False,
                       t_shaft: float | None = None, stop_mid_s: float = 0.0) -> HotMountResult:
    """stop_mid_s: the slide-on stops for this long when the face is half way over the seat (a protective stop)."""
    st, air, mo = cfg["steel"], cfg["air"], cfg["mounting"]
    T0 = mo["t_ambient"]
    Ts0 = mo["t_shaft"] if t_shaft is None else t_shaft
    alpha, k, rc = st["alpha"], st["k"], st["rho"] * st["cp"]
    d = ring.d * 1e-3
    B = ring.B * 1e-3
    lead = mo["seat_lead_mm"] * 1e-3
    # ---- inner ring nodes (leading face = node 0)
    m_ir = ring.inner_ring_mass(st["rho"])
    dz_r = B / n_ring
    c_node = m_ir * st["cp"] / n_ring
    a_cs = math.pi / 4 * ((ring.d1 * 1e-3) ** 2 - d ** 2)
    g_ax = k * a_cs / dz_r
    a_bore = math.pi * d * dz_r
    a_od = math.pi * ring.d1 * 1e-3 * dz_r
    c_rest = (ring.mass - m_ir) * st["cp"]
    a_rest = math.pi * (ring.D * 1e-3) * B + 2 * math.pi / 4 * ((ring.D * 1e-3) ** 2 - (ring.d1 * 1e-3) ** 2)
    g_ball = mo["ball_conductance_w_k"]
    t_ring = np.full(n_ring, T0 + dT)
    t_rest = T0 + dT
    # ---- shaft grid: z from -10 mm (shoulder side) to B + lead + 20 mm, r from 0 to d/2
    nr = 22
    dr = (d / 2) / nr
    rc_ = (np.arange(nr) + 0.5) * dr
    z0, z1 = -0.010, B + lead + 0.020
    dz = 0.0025
    zc = np.arange(z0 + dz / 2, z1, dz)
    nz = len(zc)
    ts = np.full((nr, nz), Ts0)
    a_sh = k / rc
    # stability of the explicit scheme
    dt = min(dt, 0.4 / (a_sh * (2 / dr ** 2 + 2 / dz ** 2)))
    vol = 2 * math.pi * rc_ * dr * dz                       # per cell (nr,)
    a_rad_face = 2 * math.pi * (rc_ + dr / 2) * dz           # radial face areas (outer face of each ring cell)
    a_ax = 2 * math.pi * rc_ * dr                            # axial face area
    g_r = k * a_rad_face[:-1] / dr                           # between i and i+1
    g_z = k * a_ax / dz                                      # between j and j+1 (per i)
    cap_sh = rc * vol[:, None] * np.ones((1, nz))
    surf_area = 2 * math.pi * (d / 2) * dz
    w_area = (rc_ * dr) / np.sum(rc_ * dr)                   # area weights for the slice mean

    def h_rad(t1, t2):
        tk1, tk2 = t1 + 273.15, t2 + 273.15
        return air["emissivity"] * SIGMA * (tk1 ** 2 + tk2 ** 2) * (tk1 + tk2)

    # ---- transfer through air (lumped, both ring parts)
    t = 0.0
    while t < t_transfer:
        step = min(dt * 10, t_transfer - t)
        tm = t_ring.mean()
        h = air["h_transfer"] + h_rad(tm, T0)
        q_ir = h * (math.pi * d * B + 2 * a_cs) * (tm - T0) + g_ball * (tm - t_rest)      # bore and both faces
        q_rest = air["h_transfer"] * a_rest * (t_rest - T0) + h_rad(t_rest, T0) * a_rest * (t_rest - T0) - g_ball * (tm - t_rest)
        t_ring = t_ring - q_ir * step / (m_ir * st["cp"])
        t_rest = t_rest - q_rest * step / c_rest
        t += step
    t_ring_at_seat = float(t_ring.mean())
    # ---- slide-on and hold
    pos_face = B + lead                       # z of the leading face; moves to 0 (shoulder)
    seated = False
    seized = False
    seize_pos = None
    min_gap = 1e9
    gap_start = None
    t_seat = None
    t_first = None
    grip_node = None
    t_full = None
    face_gap = None
    tr = {"t": [], "gap_min": [], "t_ring": [], "t_surf": [], "pos": []}
    t = 0.0
    t_slide = (B + lead) / (v_mm_s * 1e-3) + stop_mid_s
    t_end = t_slide + t_after
    stop_left = stop_mid_s
    step_i = 0
    while t < t_end:
        # node positions
        zn = pos_face + (np.arange(n_ring) + 0.5) * dz_r
        over = (zn >= 0) & (zn <= B)
        jn = np.clip(((zn - z0) / dz).astype(int), 0, nz - 1)
        tbar = (w_area[:, None] * ts).sum(axis=0)           # slice mean shaft temperature (nz,)
        gap = 0.5 * (d * 1e3 - delta_um * 1e-3 + alpha * d * 1e3 * (t_ring - T0)
                     - (d * 1e3 + alpha * d * 1e3 * (tbar[jn] - T0))) * 1e3      # um
        if over.any():
            g_over = gap[over]
            if gap_start is None:
                gap_start = float(g_over.min())
            if not seated:
                min_gap = min(min_gap, float(g_over.min()))
                if (g_over <= 0).any():
                    seized = True
                    seize_pos = float(pos_face * 1e3)
                    break
        # heat exchange ring <-> shaft surface
        gm = np.maximum(gap, 0.0) * 1e-6
        h_gap = np.where(gap > 0, air["k"] / (gm + air["gap_jump_um"] * 1e-6) + h_rad(t_ring, ts[-1, jn]), air["h_contact"])
        q = np.where(over, h_gap * a_bore * (t_ring - ts[-1, jn]), air["h_free"] * a_bore * (t_ring - T0))
        q_rest = g_ball / n_ring * (t_ring - t_rest)          # inner-ring OD faces the balls and cage, not free air
        dT_ring = -(q + q_rest) / c_node
        # axial conduction in the ring
        flux = g_ax * np.diff(t_ring)
        dT_ring[:-1] += flux / c_node
        dT_ring[1:] -= flux / c_node
        t_ring = t_ring + dT_ring * dt
        t_rest = t_rest + (float(np.sum(q_rest)) - (air["h_free"] + h_rad(t_rest, T0)) * a_rest * (t_rest - T0)) * dt / c_rest
        # shaft: conduction + heat from the ring + free convection elsewhere
        lap = np.zeros_like(ts)
        fr = g_r[:, None] * (ts[1:, :] - ts[:-1, :])
        lap[:-1, :] += fr
        lap[1:, :] -= fr
        fz = g_z[:, None] * (ts[:, 1:] - ts[:, :-1])
        lap[:, :-1] += fz
        lap[:, 1:] -= fz
        src = np.zeros(nz)
        np.add.at(src, jn[over], q[over])
        covered = np.zeros(nz, bool)
        covered[jn[over]] = True
        src[~covered] += air["h_free"] * surf_area * (T0 - ts[-1, ~covered])
        lap[-1, :] += src
        ts = ts + lap * dt / cap_sh
        t += dt
        # kinematics
        if not seated:
            if stop_left > 0 and pos_face <= B / 2:
                stop_left -= dt
            else:
                pos_face -= v_mm_s * 1e-3 * dt
            if pos_face <= 0:
                pos_face = 0.0
                seated = True
                t_seat = t
        else:
            g_now = gap
            if t_first is None and (g_now <= 0).any():
                t_first = t - t_seat
                grip_node = int(np.argmax(g_now <= 0))
                # face gap if released now: nodes between the face and the grip node shrink towards the grip node
                face_gap = float(alpha * dz_r * np.sum(t_ring[:grip_node] - T0) * 1e6 + alpha * dz_r / 2 * (t_ring[grip_node] - T0) * 1e6
                                 if grip_node > 0 else alpha * dz_r / 2 * (t_ring[0] - T0) * 1e6)
            if t_full is None and (g_now <= 0).all():
                t_full = t - t_seat
                if not keep_trace:
                    break
        if keep_trace and step_i % 25 == 0:
            tr["t"].append(t)
            tr["gap_min"].append(float(gap[over].min()) if over.any() else float("nan"))
            tr["t_ring"].append(float(t_ring.mean()))
            tr["t_surf"].append(float(ts[-1, jn].mean()))
            tr["pos"].append(pos_face * 1e3)
        step_i += 1
    return HotMountResult(seized=seized, seize_pos_mm=seize_pos, min_gap_um=float(min_gap if min_gap < 1e8 else gap_start or 0),
                          gap_at_seat_start_um=float(gap_start) if gap_start is not None else float("nan"),
                          t_first_grip_s=t_first, grip_node_from_face=grip_node, t_full_grip_s=t_full,
                          face_gap_if_released_um=face_gap, t_ring_at_seat_c=t_ring_at_seat, trace=tr)


def heating_time_s(ring: Ring, dT: float, cfg: dict) -> float:
    """Induction heating time of the whole bearing by dT (lumped energy balance)."""
    h = cfg["heater"]
    return ring.mass * cfg["steel"]["cp"] * dT / (h["power_w"] * h["efficiency"])
