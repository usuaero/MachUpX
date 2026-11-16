"""NACA 4-digit airfoils: geometry plus the section properties MachUpX needs.

Zero-lift angle and quarter-chord moment come from thin-airfoil theory applied to the
actual camber line. Maximum lift and profile drag are empirical fits for the low
Reynolds numbers small UAVs fly at (roughly 2e5 to 8e5). They are estimates: a
per-airfoil database computed with a viscous solver is the planned replacement.
"""

import math
from functools import lru_cache
from typing import Dict, List, Tuple

import numpy as np

COMMON = ["0009", "0010", "0012", "1410", "2410", "2412", "2415", "4412", "4415", "6412"]


def parse(code: str) -> Tuple[float, float, float]:
    """Returns (max camber, camber position, thickness) as fractions of chord."""
    code = code.strip().upper().replace("NACA", "").replace("_", "").replace(" ", "")
    if len(code) != 4 or not code.isdigit():
        raise ValueError(f"'{code}' is not a NACA 4-digit airfoil (for example 2412).")
    m, p, t = int(code[0]) / 100.0, int(code[1]) / 10.0, int(code[2:]) / 100.0
    if t < 0.04:
        raise ValueError("Airfoil thickness must be at least 4% of chord.")
    if m > 0 and p == 0:
        raise ValueError(f"NACA {code} has camber but no camber position.")
    return m, p, t


def _camber(x: np.ndarray, m: float, p: float) -> Tuple[np.ndarray, np.ndarray]:
    if m == 0:
        return np.zeros_like(x), np.zeros_like(x)
    fore = x < p
    yc = np.where(fore, m / p**2 * (2 * p * x - x**2), m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x**2))
    dyc = np.where(fore, 2 * m / p**2 * (p - x), 2 * m / (1 - p) ** 2 * (p - x))
    return yc, dyc


@lru_cache(maxsize=64)
def outline(code: str, n: int = 40) -> np.ndarray:
    """Closed outline, unit chord, shape (2n-1, 2): trailing edge, over the top to the
    leading edge, back along the bottom to the trailing edge. x is 0 at the leading edge."""
    m, p, t = parse(code)
    beta = np.linspace(0.0, math.pi, n)
    x = 0.5 * (1 - np.cos(beta))
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)
    yc, dyc = _camber(x, m, p)
    th = np.arctan(dyc)
    upper = np.column_stack([x - yt * np.sin(th), yc + yt * np.cos(th)])
    lower = np.column_stack([x + yt * np.sin(th), yc - yt * np.cos(th)])
    pts = np.vstack([upper[::-1], lower[1:]])
    pts[0] = pts[-1] = (1.0, 0.0)
    return pts


REFERENCE_RE = 5e5


@lru_cache(maxsize=256)
def _section(code: str, re_drag: int, re_stall: int) -> Dict[str, float]:
    m, p, t = parse(code)

    # Thin-airfoil theory integrals over the camber line.
    theta = np.linspace(0.0, math.pi, 2001)
    x = 0.5 * (1 - np.cos(theta))
    _, dyc = _camber(x, m, p)
    integrate = getattr(np, "trapezoid", None) or np.trapz
    aL0 = -integrate(dyc * (np.cos(theta) - 1), theta) / math.pi
    a1 = 2 / math.pi * integrate(dyc * np.cos(theta), theta)
    a2 = 2 / math.pi * integrate(dyc * np.cos(2 * theta), theta)
    cm_c4 = math.pi / 4 * (a2 - a1)

    # Empirical low-Reynolds fits.
    tp = t * 100
    cl_max = min(1.6, max(0.9, 1.05 + 8.0 * m + 0.02 * (min(tp, 15) - 10)))
    cl_max *= min(1.08, max(0.80, (re_stall / REFERENCE_RE) ** 0.08))
    cd_min = (0.0055 + 0.0002 * tp + 0.02 * m) * min(2.2, max(0.8, (REFERENCE_RE / re_drag) ** 0.35))
    cl_ideal = 12.0 * m
    cd2 = 0.010
    return {
        "type": "linear",
        "aL0": float(aL0),
        "CLa": 2 * math.pi * (1 + 0.77 * t) * 0.92,  # thickness gain, viscous loss
        "CmL0": float(cm_c4),
        "Cma": 0.0,
        "CD0": cd_min + cd2 * cl_ideal**2,
        "CD1": -2 * cd2 * cl_ideal,
        "CD2": cd2,
        "CL_max": cl_max,
        "geometry": {"NACA": code},
    }


def section_properties(code: str, re_drag: float = REFERENCE_RE, re_stall: float = REFERENCE_RE) -> Dict[str, float]:
    """Linear section model in MachUpX's format (angles in radians).

    re_drag is the chord Reynolds number at cruise (sets profile drag) and re_stall the one
    at stall speed (sets maximum lift). Both are rounded so results cache well.
    """
    return dict(_section(code.strip(), int(round(max(re_drag, 3e4), -4)), int(round(max(re_stall, 3e4), -4))))


def library() -> List[Dict]:
    out = []
    for code in COMMON:
        s = section_properties(code)
        out.append({
            "code": code,
            "name": f"NACA {code}",
            "cl_max": round(s["CL_max"], 2),
            "zero_lift_deg": round(math.degrees(s["aL0"]), 2),
            "cm": round(s["CmL0"], 3),
        })
    return out
