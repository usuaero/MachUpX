"""Aerodynamic analysis: turns a Design into a MachUpX model and interrogates it.

MachUpX (numerical lifting-line) supplies lift, induced drag, section profile drag,
moments and every stability, damping and control derivative. Bodies that lifting-line
theory cannot see (fuselage, booms, stopped rotors) are added as a parasite drag build-up.
"""

import math
import warnings
from typing import Dict, List, Optional

import numpy as np

import machupX as MX

from . import airfoils
from .atmosphere import G, density, reynolds, skin_friction
from .schema import Design, Surface

NAME = "uav"


# ---------------------------------------------------------------- model building

def _wing_dict(s: Surface, airfoil_name: str, wing_id: int, n: int, mixing: Optional[Dict[str, float]],
               **extra) -> Dict:
    d = {
        "ID": wing_id,
        "side": "both",
        "is_main": wing_id == 1,
        "semispan": s.span / 2,
        "chord": [[0.0, s.root_chord], [1.0, s.tip_chord]],
        "sweep": s.sweep_deg,
        "dihedral": s.dihedral_deg,
        "twist": [[0.0, s.incidence_deg], [1.0, s.incidence_deg + s.washout_deg]],
        "airfoil": airfoil_name,
        "grid": {"N": n},
    }
    if mixing:
        d["control_surface"] = {
            "root_span": s.control_span_start,
            "tip_span": s.control_span_end,
            "chord_fraction": s.control_chord_fraction,
            "control_mixing": mixing,
        }
    d.update(extra)
    return d


def section(design: Design, surface: Surface) -> Dict:
    """Section properties of a surface at this design's flight Reynolds numbers."""
    req = design.requirements
    return airfoils.section_properties(
        surface.airfoil,
        reynolds(req.cruise_speed_ms, surface.mac, req.cruise_altitude_m),
        reynolds(req.stall_speed_ms, surface.mac, req.cruise_altitude_m),
    )


def _solver_section(design: Design, surface: Surface, keep_cl_max: bool) -> Dict:
    props = section(design, surface)
    if not keep_cl_max:
        # MachUpX clips section lift at CL_max, which stalls its Newton solver. Stall is
        # detected here instead (see polar), so the solver gets the unclipped linear model.
        props.pop("CL_max")
    return props


def _clear_of_wake(design: Design) -> Dict[str, float]:
    """Vertical nudges (m) that keep tail surfaces out of the main wing's vortex sheet.

    A lifting-line control point lying on a trailing vortex sees an infinite induced
    velocity, so surfaces that sit exactly in the wing's wake plane are moved by a few
    millimetres in the aerodynamic model only. The geometry the user sees is unchanged.
    """
    w = design.wing
    tol = 0.004 * w.span
    slope = math.tan(math.radians(w.dihedral_deg))
    nudge = {"h": 0.0, "v": 0.0}
    h, v = design.htail, design.vtail
    if h:
        # Wake height above the tail, at the tail root and tip stations.
        gaps = [(w.z - y * slope) - h.z for y in (0.0, min(h.span, w.span) / 2)]
        if min(gaps) < tol and max(gaps) > -tol:
            nudge["h"] = (min(gaps) - tol) if abs(min(gaps) - tol) < abs(max(gaps) + tol) else (max(gaps) + tol)
    if v and h:  # a centreline fin, root at v.z, tip at v.z - v.span
        depth = v.z - w.z  # how far the fin root sits below the wing-root wake
        if -tol < depth < 0.3 * v.span:
            nudge["v"] = -(depth + tol)
    return nudge


def machup_airplane(design: Design, cg_x: Optional[float] = None, keep_cl_max: bool = True) -> Dict:
    """The aircraft as a MachUpX airplane dictionary (SI units)."""
    w = design.wing
    nudge = _clear_of_wake(design)
    flying_wing = design.htail is None
    names = {"wing": w, "htail": design.htail, "vtail": design.vtail}
    foils = {f"{k}_NACA_{s.airfoil}": _solver_section(design, s, keep_cl_max) for k, s in names.items() if s}

    wings = {
        "main_wing": _wing_dict(
            w, f"wing_NACA_{w.airfoil}", 1, 30,
            {"elevator": 1.0, "aileron": 1.0} if flying_wing else {"aileron": 1.0},
            connect_to={"ID": 0, "dx": w.x, "dz": w.z},
        )
    }
    controls = {"aileron": {"is_symmetric": False}, "elevator": {"is_symmetric": True}}

    if design.htail:
        h = design.htail
        v_tail = design.tail_type == "v_tail"
        # A V-tail's two panels move together for pitch and in opposition for yaw.
        mixing = {"elevator": 1.0, "rudder": 1.0} if v_tail else {"elevator": 1.0}
        wings["h_stab"] = _wing_dict(h, f"htail_NACA_{h.airfoil}", 2, 16, mixing,
                                     connect_to={"ID": 0, "dx": h.x, "dz": h.z + nudge["h"]})
        if v_tail:
            controls["rudder"] = {"is_symmetric": False}
    if design.vtail:
        v = design.vtail
        if flying_wing:  # a winglet on each tip
            wings["winglets"] = _wing_dict(
                v.model_copy(update={"span": 2 * v.span, "dihedral_deg": 90.0}), f"vtail_NACA_{v.airfoil}", 3, 10,
                None,
                connect_to={"ID": 1, "location": "tip"},
            )
        else:
            fin = _wing_dict(
                v.model_copy(update={"span": 2 * v.span, "dihedral_deg": 90.0}), f"vtail_NACA_{v.airfoil}", 3, 16,
                {"rudder": 1.0},
                connect_to={"ID": 0, "dx": v.x, "dz": v.z + nudge["v"]},
            )
            fin["side"] = "left"  # a left half-wing rolled 90 degrees points up
            wings["v_stab"] = fin
            controls["rudder"] = {"is_symmetric": False}

    return {
        "CG": [design.cg_x if cg_x is None else cg_x, 0.0, design.cg_z],
        "weight": design.mass_kg * G,
        "reference": {"area": w.area, "longitudinal_length": w.mac, "lateral_length": w.span},
        "controls": controls,
        "airfoils": foils,
        "wings": wings,
    }


def machup_scene_input(design: Design) -> Dict:
    """A complete, runnable MachUpX input for the cruise condition."""
    req = design.requirements
    return {
        "tag": f"{req.name} - generated by Wingxtra Studio",
        "run": {"solve_forces": {"non_dimensional": True}, "pitch_trim": {}, "derivatives": {}},
        "solver": {"type": "nonlinear", "constrain_vortex_sheet": True},
        "units": "SI",
        "scene": {
            "atmosphere": {"rho": density(req.cruise_altitude_m)},
            "aircraft": {NAME: {
                "file": "airplane.json",
                "state": {"velocity": req.cruise_speed_ms, "alpha": 0.0, "beta": 0.0},
                "control_state": {},
            }},
        },
    }


def make_scene(design: Design, speed: Optional[float] = None, cg_x: Optional[float] = None,
               solver: str = "nonlinear") -> MX.Scene:
    req = design.requirements
    scene = MX.Scene({
        # The wake is fixed in the body x-y plane (as in AVL) so results vary smoothly with
        # angle of attack instead of jumping when a free wake sweeps across the tail.
        "solver": {"type": solver, "constrain_vortex_sheet": True},
        "units": "SI",
        "scene": {"atmosphere": {"rho": density(req.cruise_altitude_m)}},
    })
    scene.add_aircraft(NAME, machup_airplane(design, cg_x, keep_cl_max=False),
                       state={"velocity": speed or req.cruise_speed_ms, "alpha": 0.0, "beta": 0.0})
    return scene


# ---------------------------------------------------------------- parasite drag

def parasite_buildup(design: Design, speed: Optional[float] = None) -> List[Dict]:
    """Drag areas (m^2) of everything that is not a lifting surface."""
    req = design.requirements
    V = speed or req.cruise_speed_ms
    alt = req.cruise_altitude_m
    items = []

    if design.body_cd_override is not None:
        cd = design.body_cd_override
        return [{"name": "Fuselage and other non-lifting parts (your value)", "cda": cd * design.wing.area, "cd": cd}]

    f = design.fuselage
    d_eq = math.sqrt(f.width * f.height)
    fineness = f.length / d_eq
    wetted = math.pi * d_eq * f.length * 0.75
    form = 1 + 60 / fineness**3 + fineness / 400
    items.append({"name": "Fuselage", "cda": skin_friction(reynolds(V, f.length, alt)) * form * wetted})

    r = design.lift_rotors
    if r:
        boom_len = r.front_x - r.rear_x
        cf = skin_friction(reynolds(V, boom_len, alt))
        items.append({"name": "Rotor booms", "cda": 2 * cf * 1.15 * math.pi * r.boom_diameter * boom_len})
        items.append({"name": "Stopped lift rotors", "cda": r.count * 0.012 * r.diameter**2})

    # Real airframes carry hinge gaps, control horns, antennas, hatches and surface
    # roughness that no clean-geometry method sees: 12% of all zero-lift drag.
    surfaces = [(design.wing, 1), (design.htail, 1), (design.vtail, 2 if design.htail is None else 1)]
    profile = sum(n * s.area * section(design, s)["CD0"] for s, n in surfaces if s)
    subtotal = sum(i["cda"] for i in items)
    items.append({"name": "Gaps, linkages, antennas, roughness (12%)", "cda": 0.12 * (subtotal + profile)})
    for i in items:
        i["cd"] = i["cda"] / design.wing.area
    return items


def parasite_cd(design: Design, speed: Optional[float] = None) -> float:
    return sum(i["cd"] for i in parasite_buildup(design, speed))


# ---------------------------------------------------------------- analyses

def _floats(d: Dict) -> Dict:
    return {k: float(v) for k, v in d.items()}


def _main_wing_sections(scene: MX.Scene, clmax: float):
    dist = scene.distributions(verbose=False)[NAME]
    seg = dist["main_wing_right"]
    return np.array(seg["span_frac"]), np.array(seg["section_CL"]), np.array(seg["chord"]), np.array(seg["cpy"])


def polar(design: Design, scene: Optional[MX.Scene] = None) -> Dict:
    """Untrimmed alpha sweep with a critical-section stall prediction.

    Stall is declared when the most heavily loaded section of the main wing reaches its
    airfoil's maximum lift coefficient; the sweep stops there because lifting-line results
    are not meaningful beyond it.
    """
    scene = scene or make_scene(design)  # nonlinear: the linear solver's section loads are unreliable
    clmax_section = section(design, design.wing)["CL_max"]
    cd_extra = parasite_cd(design)
    rows, peak_prev, stall = [], None, None

    for alpha in np.arange(-4.0, 22.01, 1.0):
        scene.set_aircraft_state(state={"velocity": design.requirements.cruise_speed_ms, "alpha": float(alpha)})
        try:
            total = scene.solve_forces(non_dimensional=True, dimensional=False, verbose=False)[NAME]["total"]
        except Exception:
            continue  # an isolated non-converged point is skipped, not fatal
        span_frac, cl_sec, _, _ = _main_wing_sections(scene, clmax_section)
        i = int(np.argmax(cl_sec))
        peak = float(cl_sec[i])
        row = {"alpha": float(alpha), "CL": float(total["CL"]), "CD": float(total["CD"]) + cd_extra,
               "Cm": float(total["Cm"]), "peak_section_cl": peak}
        if peak >= clmax_section and rows:
            prev = rows[-1]
            t = (clmax_section - peak_prev) / (peak - peak_prev)
            stall = {k: prev[k] + t * (row[k] - prev[k]) for k in ("alpha", "CL", "CD", "Cm")}
            stall["span_fraction"] = float(span_frac[i])
            rows.append({**stall, "peak_section_cl": clmax_section})
            break
        rows.append(row)
        peak_prev = peak

    if not rows:
        raise RuntimeError("The aerodynamic solver did not converge for this geometry.")
    if stall is None:  # never reached: fall back to the last point
        last = rows[-1]
        stall = {**{k: last[k] for k in ("alpha", "CL", "CD", "Cm")}, "span_fraction": 0.0}
    for r in rows:
        r["LD"] = r["CL"] / r["CD"] if r["CD"] > 0 else 0.0

    # Quadratic drag polar over the useful lift range, for performance work.
    pts = [r for r in rows if r["CL"] > -0.1]
    c2, c1, c0 = np.polyfit([r["CL"] for r in pts], [r["CD"] for r in pts], 2)
    best = max(rows, key=lambda r: r["LD"])
    return {
        "points": rows,
        "CL_max": stall["CL"],
        "stall_alpha": stall["alpha"],
        "stall_span_fraction": stall["span_fraction"],
        "section_cl_max": clmax_section,
        "drag_fit": [float(c0), float(c1), float(c2)],
        "LD_max": best["LD"],
        "CL_best_LD": best["CL"],
        "CD_parasite_extra": cd_extra,
    }


def neutral_point(design: Design) -> float:
    """x location of the stick-fixed neutral point (positive forward of the datum)."""
    scene = make_scene(design, cg_x=0.0, solver="linear")
    scene.set_aircraft_state(state={"velocity": design.requirements.cruise_speed_ms, "alpha": 2.0})
    d = scene.stability_derivatives()[NAME]
    return float(design.wing.mac * d["Cm,a"] / d["CL,a"])


def trim_and_derivatives(design: Design, cl_max: float) -> Dict:
    """Cruise trim, lift distribution, all derivatives, and elevator needed near stall.

    Uses the full nonlinear lifting-line solution, falling back to the linearised
    solution if Newton's method does not converge for an unusual geometry.
    """
    try:
        return {**_trim_and_derivatives(design, cl_max, "nonlinear"), "solver": "nonlinear"}
    except Exception:
        return {**_trim_and_derivatives(design, cl_max, "linear"), "solver": "linear"}


def _trim_and_derivatives(design: Design, cl_max: float, solver: str) -> Dict:
    req = design.requirements
    scene = make_scene(design, solver=solver)
    rho = density(req.cruise_altitude_m)
    q = 0.5 * rho * req.cruise_speed_ms**2
    cl_cruise = design.mass_kg * G / (q * design.wing.area)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        trim = scene.pitch_trim(set_trim_state=True, verbose=False)[NAME]
        total = scene.solve_forces(non_dimensional=True, dimensional=False, verbose=False)[NAME]["total"]
        span_frac, cl_sec, chord, cpy = _main_wing_sections(scene, 0)
        derivs = scene.derivatives()[NAME]

        slow = None
        try:
            slow_scene = make_scene(design, solver=solver)
            t = slow_scene.pitch_trim(CL=0.95 * cl_max, set_trim_state=False, verbose=False)[NAME]
            slow = {"alpha": float(t["alpha"]), "elevator": float(t["elevator"])}
        except Exception:
            pass

    load = cl_sec * chord
    semi = design.wing.span / 2
    y = np.abs(cpy)
    order = np.argsort(y)
    ellipse = np.sqrt(np.clip(1 - (y[order] / semi) ** 2, 0, 1))
    scale = np.trapezoid(load[order], y[order]) / max(np.trapezoid(ellipse, y[order]), 1e-9)

    stab, damp, ctrl = _floats(derivs["stability"]), _floats(derivs["damping"]), _floats(derivs["control"])
    cd_extra = parasite_cd(design)
    return {
        "alpha": float(trim["alpha"]),
        "elevator": float(trim["elevator"]),
        "CL": float(total["CL"]),
        "CD": float(total["CD"]) + cd_extra,
        "LD": float(total["CL"]) / (float(total["CD"]) + cd_extra),
        "CL_required": cl_cruise,
        "static_margin": -stab["Cm,a"] / stab["CL,a"],
        "neutral_point_x": design.cg_x + design.wing.mac * stab["Cm,a"] / stab["CL,a"],
        "near_stall": slow,
        "stability": stab,
        "damping": damp,
        "control": ctrl,
        "lift_distribution": {
            "y": [float(v) for v in y[order]],
            "section_cl": [float(v) for v in cl_sec[order]],
            "load": [float(v) for v in load[order]],
            "elliptical": [float(v) for v in ellipse * scale],
        },
    }
