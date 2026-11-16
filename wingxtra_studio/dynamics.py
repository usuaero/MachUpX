"""Flight dynamics: linearised equations of motion about cruise trim.

The classical small-disturbance model (Etkin; Phillips, Mechanics of Flight) is assembled
from the lifting-line derivatives and the estimated inertia, giving the five natural
modes and time-domain responses to control inputs.
"""

import math
from typing import Dict, List

import numpy as np
from scipy import signal

from .atmosphere import G, density
from .schema import Design


def _mode(eigs: np.ndarray) -> Dict:
    """Describes one mode from its eigenvalue (real) or conjugate pair."""
    lam = eigs[0]
    sigma = float(lam.real)
    out = {"real": sigma, "imag": float(abs(lam.imag)), "stable": sigma < 0}
    if abs(sigma) > 1e-9:
        out["time_to_half_or_double_s"] = math.log(2) / abs(sigma)
    if abs(lam.imag) > 1e-9:
        wn = abs(lam)
        out.update(oscillatory=True, natural_frequency_rad_s=float(wn), damping_ratio=float(-sigma / wn),
                   period_s=float(2 * math.pi / abs(lam.imag)))
    else:
        out.update(oscillatory=False, time_constant_s=(1 / abs(sigma) if abs(sigma) > 1e-9 else None))
    return out


def _split(eigs: np.ndarray):
    pairs = sorted([e for e in eigs if e.imag > 1e-9], key=abs, reverse=True)
    reals = sorted([e.real for e in eigs if abs(e.imag) <= 1e-9])
    return pairs, reals


def _series(t: np.ndarray, cols: Dict[str, np.ndarray], keep: int = 240) -> Dict[str, List[float]]:
    step = max(1, len(t) // keep)
    out = {"t": [float(v) for v in t[::step]]}
    out.update({k: [float(x) for x in v[::step]] for k, v in cols.items()})
    return out


def dynamics(design: Design, trim: Dict) -> Dict:
    req = design.requirements
    rho = density(req.cruise_altitude_m)
    V = req.cruise_speed_ms
    m, S, c, b = design.mass_kg, design.wing.area, design.wing.mac, design.wing.span
    Q = 0.5 * rho * V * V
    I = design.inertia
    st, dm, ct = trim["stability"], trim["damping"], trim["control"]
    CL, CD = trim["CL"], trim["CD"]

    # ---------------- longitudinal: states u, w, q, theta
    Xu = -2 * CD * Q * S / (m * V)
    Xw = (CL - st["CD,a"]) * Q * S / (m * V)
    Zu = -2 * CL * Q * S / (m * V)
    Zw = -(st["CL,a"] + CD) * Q * S / (m * V)
    Zq = -dm["CL,qbar"] * (c / (2 * V)) * Q * S / m
    Mw = st["Cm,a"] * Q * S * c / (V * I["Iyy"])
    Mq = dm["Cm,qbar"] * (c / (2 * V)) * Q * S * c / I["Iyy"]
    # Lag-of-downwash damping is not available from a steady solver; for a tailed aircraft
    # it is close to one third of the pitch damping. A flying wing has none.
    cm_adot = dm["Cm,qbar"] / 3 if design.htail else 0.0
    Mwd = cm_adot * (c / (2 * V)) * Q * S * c / (V * I["Iyy"])
    Zde = -ct["CL,delevator"] * Q * S / m
    Mde = ct["Cm,delevator"] * Q * S * c / I["Iyy"]

    A_lon = np.array([
        [Xu, Xw, 0.0, -G],
        [Zu, Zw, V + Zq, 0.0],
        [Mwd * Zu, Mw + Mwd * Zw, Mq + Mwd * (V + Zq), 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ])
    B_lon = np.array([[0.0], [Zde], [Mde + Mwd * Zde], [0.0]])

    pairs, reals = _split(np.linalg.eigvals(A_lon))
    modes = {}
    if len(pairs) == 2:
        modes["short_period"] = _mode(np.array([pairs[0]]))
        modes["phugoid"] = _mode(np.array([pairs[1]]))
    elif len(pairs) == 1:
        # One pair has broken into two real roots (very heavy damping or an instability).
        fast = [r for r in reals]
        if abs(pairs[0]) > max(abs(r) for r in fast):
            modes["short_period"] = _mode(np.array([pairs[0]]))
            modes["phugoid"] = _mode(np.array([complex(max(fast))]))
        else:
            modes["phugoid"] = _mode(np.array([pairs[0]]))
            modes["short_period"] = _mode(np.array([complex(max(fast))]))
    else:
        modes["short_period"] = _mode(np.array([complex(reals[0])]))
        modes["phugoid"] = _mode(np.array([complex(reals[-1])]))

    # ---------------- lateral: states beta, p, r, phi
    Yb = Q * S * st["Cy,b"] / m
    Yp = Q * S * b * dm["Cy,pbar"] / (2 * m * V)
    Yr = Q * S * b * dm["Cy,rbar"] / (2 * m * V)
    Lb = Q * S * b * st["Cl,b"] / I["Ixx"]
    Lp = Q * S * b * b * dm["Cl,pbar"] / (2 * I["Ixx"] * V)
    Lr = Q * S * b * b * dm["Cl,rbar"] / (2 * I["Ixx"] * V)
    Nb = Q * S * b * st["Cn,b"] / I["Izz"]
    Np = Q * S * b * b * dm["Cn,pbar"] / (2 * I["Izz"] * V)
    Nr = Q * S * b * b * dm["Cn,rbar"] / (2 * I["Izz"] * V)

    A_lat = np.array([
        [Yb / V, Yp / V, Yr / V - 1.0, G / V],
        [Lb, Lp, Lr, 0.0],
        [Nb, Np, Nr, 0.0],
        [0.0, 1.0, 0.0, 0.0],
    ])
    B_lat = np.array([
        [Q * S * ct.get("Cy,daileron", 0.0) / (m * V)],
        [Q * S * b * ct["Cl,daileron"] / I["Ixx"]],
        [Q * S * b * ct["Cn,daileron"] / I["Izz"]],
        [0.0],
    ])

    pairs, reals = _split(np.linalg.eigvals(A_lat))
    if pairs:
        modes["dutch_roll"] = _mode(np.array([pairs[0]]))
    if len(reals) >= 2:
        modes["roll"] = _mode(np.array([complex(reals[0])]))
        modes["spiral"] = _mode(np.array([complex(reals[-1])]))

    # ---------------- time responses
    # Elevator doublet: 2 degrees nose-up for half a second, then nose-down, then released.
    t = np.arange(0.0, 40.0, 0.02)
    de = np.radians(np.where((t >= 1.0) & (t < 1.5), -2.0, np.where((t >= 1.5) & (t < 2.0), 2.0, 0.0)))
    _, _, x = signal.lsim(signal.StateSpace(A_lon, B_lon, np.eye(4), np.zeros((4, 1))), de, t)
    climb = V * (x[:, 3] - x[:, 1] / V)
    pitch = _series(t, {
        "elevator_deg": np.degrees(de), "pitch_deg": np.degrees(x[:, 3]), "alpha_deg": np.degrees(x[:, 1] / V),
        "speed_ms": V + x[:, 0], "pitch_rate_dps": np.degrees(x[:, 2]),
        "altitude_m": np.cumsum(climb) * 0.02,
    })

    # Aileron pulse: 2 degrees for half a second, then released.
    t2 = np.arange(0.0, 20.0, 0.02)
    da = np.radians(np.where((t2 >= 1.0) & (t2 < 1.5), 2.0, 0.0))
    _, _, y = signal.lsim(signal.StateSpace(A_lat, B_lat, np.eye(4), np.zeros((4, 1))), da, t2)
    # Report roll in the direction the stick was moved, whatever the solver's sign convention.
    sign = 1.0 if ct["Cl,daileron"] >= 0 else -1.0
    roll = _series(t2, {
        "aileron_deg": np.degrees(da), "bank_deg": sign * np.degrees(y[:, 3]),
        "roll_rate_dps": sign * np.degrees(y[:, 1]), "sideslip_deg": sign * np.degrees(y[:, 0]),
        "yaw_rate_dps": sign * np.degrees(y[:, 2]),
    })

    steady_roll_rate = abs(B_lat[1, 0] / Lp) if abs(Lp) > 1e-9 else 0.0  # rad/s per rad of aileron
    return {
        "modes": modes,
        "responses": {"pitch_doublet": pitch, "roll_pulse": roll},
        "roll_rate_dps_at_full_aileron": math.degrees(steady_roll_rate * math.radians(20.0)),
        "matrices": {"A_longitudinal": A_lon.tolist(), "B_longitudinal": B_lon.tolist(),
                     "A_lateral": A_lat.tolist(), "B_lateral": B_lat.tolist()},
    }
