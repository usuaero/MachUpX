"""3D geometry: section outlines shared by the on-screen mesh and the CAD export.

Everything is produced as "lofts": ordered lists of closed section rings with the same
point count. A viewer triangulates between rings; the STEP exporter lofts through them.
"""

import math
from typing import Dict, List

import numpy as np

from . import airfoils
from .schema import Design, Surface

RING = 28  # points around a body cross-section


def _airfoil_ring(code: str, quarter_chord: np.ndarray, chord: float, twist_deg: float,
                  normal: np.ndarray, n: int = 30) -> np.ndarray:
    pts = airfoils.outline(code, n)
    u = (0.25 - pts[:, 0]) * chord  # forward of the quarter chord
    h = pts[:, 1] * chord  # above the chord line
    t = math.radians(twist_deg)
    u2, h2 = u * math.cos(t) - h * math.sin(t), u * math.sin(t) + h * math.cos(t)
    return quarter_chord + np.outer(u2, [1.0, 0.0, 0.0]) + np.outer(h2, normal)


def surface_loft(s: Surface, side: int = 1, vertical: bool = False, origin=None, half_span=None) -> List[np.ndarray]:
    """Root and tip rings of one half of a surface (side=+1 right, -1 left)."""
    root = np.array([s.x, 0.0, s.z]) if origin is None else np.asarray(origin, float)
    if vertical:
        span_dir, normal, length = np.array([0.0, 0.0, -1.0]), np.array([0.0, float(side), 0.0]), s.span
    else:
        g = math.radians(s.dihedral_deg)
        span_dir = np.array([0.0, side * math.cos(g), -math.sin(g)])
        normal = np.array([0.0, -side * math.sin(g), -math.cos(g)])
        length = s.span / 2
    if half_span is not None:
        length = half_span
    rings = []
    for eta, chord, twist in ((0.0, s.root_chord, s.incidence_deg), (1.0, s.tip_chord, s.incidence_deg + s.washout_deg)):
        qc = root + eta * length * span_dir + np.array([-eta * length * math.tan(math.radians(s.sweep_deg)), 0, 0])
        rings.append(_airfoil_ring(s.airfoil, qc, chord, twist, normal))
    return rings


def wing_tip_point(s: Surface, side: int) -> np.ndarray:
    g = math.radians(s.dihedral_deg)
    half = s.span / 2
    return np.array([s.x - half * math.tan(math.radians(s.sweep_deg)), side * half * math.cos(g), s.z - half * math.sin(g)])


def _ellipse_ring(x: float, y: float, z: float, ry: float, rz: float) -> np.ndarray:
    a = np.linspace(0, 2 * math.pi, RING, endpoint=False)
    return np.column_stack([np.full(RING, x), y + ry * np.cos(a), z + rz * np.sin(a)])


def fuselage_loft(design: Design) -> List[np.ndarray]:
    f = design.fuselage
    tail_scale = 0.28 if design.htail else 0.45
    rings = []
    for xi in (0.0, 0.03, 0.08, 0.16, 0.28, 0.45, 0.62, 0.8, 0.92, 1.0):
        if xi < 0.28:
            r = 0.12 + 0.88 * math.sqrt(1 - (1 - xi / 0.28) ** 2)
        elif xi < 0.45:
            r = 1.0
        else:
            k = (xi - 0.45) / 0.55
            r = 1.0 - (1 - tail_scale) * (3 * k * k - 2 * k**3)
        # Keep the upper surface straight so a high wing and the tail sit on the body.
        rz = 0.5 * f.height * r
        rings.append(_ellipse_ring(f.nose_x - xi * f.length, 0.0, f.z - (0.5 * f.height - rz) * 0.6, 0.5 * f.width * r, rz))
    return rings


def _cylinder_x(x0: float, x1: float, y: float, z: float, radius: float) -> List[np.ndarray]:
    return [_ellipse_ring(x0, y, z, radius, radius), _ellipse_ring(x1, y, z, radius, radius)]


def _disc_z(x: float, y: float, z: float, radius: float, thickness: float = 0.004) -> List[np.ndarray]:
    a = np.linspace(0, 2 * math.pi, RING, endpoint=False)
    ring = lambda zz: np.column_stack([x + radius * np.cos(a), y + radius * np.sin(a), np.full(RING, zz)])
    return [ring(z), ring(z - thickness)]


def lofts(design: Design) -> List[Dict]:
    """Every solid of the aircraft as {name, kind, rings, ruled}."""
    out = []
    w = design.wing
    for side, tag in ((1, "right"), (-1, "left")):
        out.append({"name": f"Wing {tag}", "kind": "wing", "rings": surface_loft(w, side), "ruled": True})
    if design.htail:
        for side, tag in ((1, "right"), (-1, "left")):
            out.append({"name": f"{'V-tail' if design.tail_type == 'v_tail' else 'Horizontal tail'} {tag}", "kind": "tail", "rings": surface_loft(design.htail, side), "ruled": True})
        if design.vtail:
            out.append({"name": "Fin", "kind": "tail", "rings": surface_loft(design.vtail, 1, vertical=True), "ruled": True})
    elif design.vtail:
        for side, tag in ((1, "right"), (-1, "left")):
            out.append({"name": f"Winglet {tag}", "kind": "tail", "ruled": True,
                        "rings": surface_loft(design.vtail, side, vertical=True, origin=wing_tip_point(w, side))})

    out.append({"name": "Fuselage", "kind": "body", "rings": fuselage_loft(design), "ruled": False})

    p = design.propulsion
    x0 = p.prop_x
    out.append({"name": "Propeller disc", "kind": "prop", "ruled": True,
                "rings": [_ellipse_ring(x0, 0, design.fuselage.z, p.prop_diameter / 2, p.prop_diameter / 2),
                          _ellipse_ring(x0 + (0.004 if p.layout == "tractor" else -0.004), 0, design.fuselage.z,
                                        p.prop_diameter / 2, p.prop_diameter / 2)]})

    r = design.lift_rotors
    if r:
        for side, tag in ((1, "right"), (-1, "left")):
            y = side * r.boom_y
            out.append({"name": f"Rotor boom {tag}", "kind": "boom", "ruled": True,
                        "rings": _cylinder_x(r.front_x + 0.03, r.rear_x - 0.03, y, r.z, r.boom_diameter / 2)})
            for x, where in ((r.front_x, "front"), (r.rear_x, "rear")):
                out.append({"name": f"Lift rotor {where} {tag}", "kind": "rotor", "ruled": True,
                            "rings": _disc_z(x, y, r.z - r.boom_diameter / 2 - 0.03, r.diameter / 2)})
    return out


def _triangulate(rings: List[np.ndarray]):
    n = len(rings[0])
    closed = np.allclose(rings[0][0], rings[0][-1])  # airfoil rings repeat the trailing-edge point
    verts = np.vstack(rings)
    idx = []
    for k in range(len(rings) - 1):
        a, b = k * n, (k + 1) * n
        for i in range(n - 1 if closed else n):
            j = (i + 1) % n
            idx += [a + i, b + i, b + j, a + i, b + j, a + j]
    # End caps as fans about each end ring's centroid.
    for k, flip in ((0, False), (len(rings) - 1, True)):
        base = k * n
        centre = len(verts)
        verts = np.vstack([verts, rings[k].mean(axis=0)])
        for i in range(n - 1 if closed else n):
            j = (i + 1) % n
            idx += [centre, base + j, base + i] if flip else [centre, base + i, base + j]
    return verts, idx


def mesh(design: Design) -> Dict:
    """Triangle meshes for the viewer, in metres, body axes (x forward, y right, z down)."""
    parts = []
    for loft in lofts(design):
        verts, idx = _triangulate(loft["rings"])
        parts.append({"name": loft["name"], "kind": loft["kind"],
                      "positions": [round(float(v), 5) for v in verts.ravel()], "indices": idx})
    w = design.wing
    extent = max(w.span, design.fuselage.length)
    return {"parts": parts, "extent": extent, "cg": [design.cg_x, 0.0, design.cg_z],
            "masses": [{"name": m.name, "mass_kg": m.mass_kg, "position": [m.x, m.y, m.z]} for m in design.masses]}
