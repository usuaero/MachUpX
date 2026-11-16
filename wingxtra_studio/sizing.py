"""Conceptual sizing: from mission requirements to a complete, balanced aircraft.

The loop is closed on the analysis, not on rules of thumb: after a first-guess sizing
the aircraft is run through the lifting-line solver, and the predicted maximum lift and
drag polar are fed back until the stall speed and battery match what the solver predicts.
"""

import math
from typing import Dict, List, Optional, Tuple

from . import aero, airfoils
from .atmosphere import G, density
from .schema import Design, Fuselage, LiftRotors, MassItem, Propulsion, Requirements, Surface

FIGURE_OF_MERIT = 0.65
HOVER_THRUST_MARGIN = 1.6  # peak thrust / weight for control authority in hover


class SizingError(ValueError):
    """The requirements cannot be met; the message says why and what to change."""


def _first_guess(req: Requirements) -> Dict[str, float]:
    cd0 = {"conventional": 0.030, "flying_wing": 0.020, "vtol_quadplane": 0.042}[req.config]
    return {
        "cl_max": 0.85 * airfoils.section_properties(req.wing_airfoil)["CL_max"],
        "c0": cd0, "c1": 0.0, "c2": 1.0 / (math.pi * 0.8 * req.aspect_ratio),
    }


def hover_power(req: Requirements, mass: float) -> Tuple[float, float]:
    """(electrical hover power in W, rotor diameter in m) for a four-rotor lift system."""
    rho = density(req.cruise_altitude_m)
    weight = mass * G
    ideal = weight * math.sqrt(req.rotor_disk_loading_nm2 / (2 * rho))
    diameter = math.sqrt(4 * weight / (4 * math.pi * req.rotor_disk_loading_nm2))
    return ideal / FIGURE_OF_MERIT / req.drive_efficiency, diameter


def _mass_loop(req: Requirements, model: Dict[str, float]) -> Dict[str, float]:
    """Fixed-point iteration on take-off mass."""
    rho = density(req.cruise_altitude_m)
    q = 0.5 * rho * req.cruise_speed_ms**2
    q_stall = 0.5 * rho * req.stall_speed_ms**2
    vtol = req.config == "vtol_quadplane"
    usable_j_per_kg = req.battery_wh_per_kg * 3600 * req.battery_usable_fraction
    mass = max(2.0, 3 * (req.payload_kg + req.avionics_kg))

    for _ in range(400):
        weight = mass * G
        area = weight / (q_stall * model["cl_max"])
        cl = weight / (q * area)
        cd = model["c0"] + model["c1"] * cl + model["c2"] * cl * cl
        drag = q * area * cd
        shaft_cruise = drag * req.cruise_speed_ms / req.prop_efficiency
        elec_cruise = shaft_cruise / req.drive_efficiency + req.avionics_power_w
        shaft_max = max((drag * req.cruise_speed_ms + weight * req.climb_rate_ms) / req.prop_efficiency,
                        1.5 * shaft_cruise)
        energy = elec_cruise * req.endurance_min * 60

        cruise_system = 0.05 + 0.45 * shaft_max / 1000
        lift_system = hover_w = lift_motor_w = 0.0
        if vtol:
            hover_w, _ = hover_power(req, mass)
            energy += hover_w * req.hover_time_min * 60
            lift_motor_w = hover_w * req.drive_efficiency * HOVER_THRUST_MARGIN**1.5 / 4
            lift_system = 4 * (0.03 + 0.30 * lift_motor_w / 1000) + 0.04 * mass

        battery = energy / usable_j_per_kg
        new = (req.payload_kg + req.avionics_kg + battery + cruise_system + lift_system
               + req.structure_fraction * mass)
        if new > 600 or not math.isfinite(new):
            break
        if abs(new - mass) < 1e-6:
            return {
                "mass": new, "area": area, "battery": battery, "battery_wh": energy / 3600 / req.battery_usable_fraction,
                "cruise_system": cruise_system, "lift_system": lift_system, "shaft_max": shaft_max,
                "hover_w": hover_w, "lift_motor_w": lift_motor_w, "structure": req.structure_fraction * new,
                "cruise_cl": cl,
            }
        mass += 0.5 * (new - mass)

    raise SizingError(
        "These requirements do not close: the battery needed to fly the mission weighs more than the "
        "aircraft can carry. Try a shorter endurance, a lower cruise speed, a higher aspect ratio, "
        "a lighter payload, or a higher battery energy density."
    )


def _geometry(req: Requirements, m: Dict[str, float]) -> Design:
    """Lays out surfaces, body and propulsion for the converged mass and wing area."""
    S, AR, lam = m["area"], req.aspect_ratio, req.taper_ratio
    b = math.sqrt(AR * S)
    c_root = 2 * S / (b * (1 + lam))
    section = airfoils.section_properties(req.wing_airfoil)
    lift_slope = section["CLa"] / (1 + section["CLa"] / (math.pi * AR))
    incidence = max(0.0, min(6.0, math.degrees(m["cruise_cl"] / lift_slope + section["aL0"])))
    prop_d = 0.43 * (m["shaft_max"] / 1000) ** 0.25

    if req.config == "flying_wing":
        wing = Surface(span=b, root_chord=c_root, tip_chord=c_root * lam, sweep_deg=24.0, dihedral_deg=1.0,
                       incidence_deg=incidence, washout_deg=-4.5, airfoil=req.wing_airfoil,
                       control_chord_fraction=0.25, control_span_start=0.35, control_span_end=0.95)
        fin_area = 0.035 * S  # each winglet
        fin_h = math.sqrt(1.6 * fin_area)
        fin_root = 2 * fin_area / (fin_h * 1.6)
        vtail = Surface(span=fin_h, root_chord=min(fin_root, wing.tip_chord), tip_chord=0.6 * fin_root,
                        sweep_deg=25.0, airfoil="0009", x=-(b / 2) * math.tan(math.radians(24.0)))
        pod_len = 1.7 * c_root
        width = min(0.3, max(0.08, 0.45 * c_root))
        fuselage = Fuselage(length=pod_len, width=width, height=0.8 * width, nose_x=0.25 * c_root + 0.55 * c_root)
        prop_x = fuselage.nose_x - pod_len - 0.02
        return Design(requirements=req, mass_kg=m["mass"], wing=wing, vtail=vtail, fuselage=fuselage,
                      propulsion=Propulsion(layout="pusher", prop_diameter=prop_d, motor_power_w=m["shaft_max"],
                                            prop_x=prop_x), battery_wh=m["battery_wh"])

    wing = Surface(span=b, root_chord=c_root, tip_chord=c_root * lam, sweep_deg=0.0, dihedral_deg=3.0,
                   incidence_deg=incidence, washout_deg=-2.0, airfoil=req.wing_airfoil,
                   control_chord_fraction=0.25, control_span_start=0.55, control_span_end=0.95)
    mac = wing.mac
    arm = min(4.5 * mac, max(2.6 * mac, 0.42 * b))

    def tail(area: float, ar: float, taper: float, height_only: bool) -> Tuple[float, float, float]:
        span = math.sqrt(ar * area)
        root = 2 * area / (span * (1 + taper))
        return span, root, root * taper

    hs, hr, ht = tail(0.50 * S * mac / arm, 4.0, 0.7, False)
    vs, vr, vt = tail(0.035 * S * b / arm, 1.5, 0.55, True)

    width = min(0.45, max(0.08, 0.5 * mac))
    height = 1.1 * width
    nose_len = max(0.16 * b, 1.3 * c_root)
    nose_x = 0.25 * c_root + nose_len
    length = nose_x + arm + 0.8 * hr
    wing.z = -0.4 * height
    fuselage = Fuselage(length=length, width=width, height=height, nose_x=nose_x)
    htail = Surface(span=hs, root_chord=hr, tip_chord=ht, airfoil="0010", x=-arm, z=-0.1 * height,
                    sweep_deg=5.0, control_chord_fraction=0.35, control_span_start=0.0, control_span_end=1.0)
    vtail = Surface(span=vs, root_chord=vr, tip_chord=vt, airfoil="0010", x=-arm, z=-0.5 * height,
                    sweep_deg=20.0, control_chord_fraction=0.35, control_span_start=0.0, control_span_end=1.0)
    tail_type = "conventional"
    if req.tail == "v_tail":
        htail, vtail, tail_type = v_tail(htail.area, vtail.area, arm, z=-0.3 * height), None, "v_tail"
    return Design(requirements=req, mass_kg=m["mass"], wing=wing, htail=htail, vtail=vtail, fuselage=fuselage,
                  tail_type=tail_type,
                  propulsion=Propulsion(layout="tractor", prop_diameter=prop_d, motor_power_w=m["shaft_max"],
                                        prop_x=nose_x + 0.02, hover_power_w=m["hover_w"],
                                        lift_motor_power_w=m["lift_motor_w"]),
                  battery_wh=m["battery_wh"])


def v_tail(h_area: float, v_area: float, arm: float, z: float = 0.0, span: Optional[float] = None,
           taper: float = 0.8, average_chord: Optional[float] = None) -> Surface:
    """Replaces a horizontal tail and fin with a V-tail of the same projected areas.

    Total area is the sum of the two; the dihedral angle is atan(sqrt(S_v / S_h)), so each
    projection matches the surface it replaces. Span is measured along the panels.
    """
    area = h_area + v_area
    dihedral = math.degrees(math.atan(math.sqrt(v_area / h_area)))
    if span is None:
        span = math.sqrt(4.5 * area)
    chord = average_chord if average_chord is not None else area / span
    root = 2 * chord / (1 + taper)
    return Surface(span=span, root_chord=root, tip_chord=root * taper, dihedral_deg=dihedral, airfoil="0010",
                   x=-arm, z=z, sweep_deg=5.0, control_chord_fraction=0.35)


def mac_quarter_chord_x(s: Surface) -> float:
    y_mac = s.span / 6 * (1 + 2 * s.taper) / (1 + s.taper)
    return s.x - y_mac * math.tan(math.radians(s.sweep_deg))


def balance(design: Design, m: Dict[str, float], target_cg_x: float) -> None:
    """Places every mass, solves the battery position for the target CG, estimates inertia."""
    req = design.requirements
    f, w = design.fuselage, design.wing
    tail_end = f.nose_x - f.length
    mid = f.nose_x - 0.5 * f.length
    struct = m["structure"]
    items: List[MassItem] = []

    wing_x = mac_quarter_chord_x(w) - 0.15 * w.mac
    if design.htail:
        items += [MassItem(name="Wing structure", mass_kg=0.45 * struct, x=wing_x, z=w.z),
                  MassItem(name="Fuselage structure", mass_kg=0.37 * struct, x=mid),
                  MassItem(name="Tail structure", mass_kg=0.18 * struct, x=design.htail.x)]
    else:
        items += [MassItem(name="Wing structure", mass_kg=0.82 * struct, x=wing_x, z=w.z),
                  MassItem(name="Centre pod structure", mass_kg=0.18 * struct, x=mid)]

    motor_x = design.propulsion.prop_x + (-0.04 if design.propulsion.layout == "tractor" else 0.04)
    items.append(MassItem(name="Cruise motor, ESC, propeller", mass_kg=m["cruise_system"], x=motor_x))

    r = design.lift_rotors
    if r:
        items.append(MassItem(name="Lift motors, ESCs, rotors, booms", mass_kg=m["lift_system"],
                              x=0.5 * (r.front_x + r.rear_x), z=r.z,
                              note="Four lift units, symmetric about the centre of gravity"))
    items.append(MassItem(name="Payload", mass_kg=req.payload_kg, x=target_cg_x,
                          note="Placed on the centre of gravity so payload changes do not affect trim"))
    items.append(MassItem(name="Avionics", mass_kg=req.avionics_kg, x=target_cg_x + 0.3 * w.mac))

    fixed_mass = sum(i.mass_kg for i in items)
    fixed_moment = sum(i.mass_kg * i.x for i in items)
    battery = m["battery"]
    want = (design.mass_kg * target_cg_x - fixed_moment) / battery
    lo, hi = tail_end + 0.15 * f.length, f.nose_x - 0.08 * f.length
    battery_x = min(hi, max(lo, want))
    note = "Position solved to put the centre of gravity on target"
    if abs(battery_x - want) > 1e-6:
        note = (f"Cannot reach the target centre of gravity: the battery would need to sit "
                f"{abs(want - battery_x) * 1000:.0f} mm outside the usable bay")
        design.notes.append(note + ". Move the wing, lengthen the nose or add ballast.")
    items.append(MassItem(name="Battery", mass_kg=battery, x=battery_x, note=note))

    total = fixed_mass + battery
    design.masses = items
    design.cg_x = (fixed_moment + battery * battery_x) / total
    design.cg_z = sum(i.mass_kg * i.z for i in items) / total

    # Inertia: point masses about the CG plus the spread of the distributed structure.
    b = w.span
    wing_mass = items[0].mass_kg
    ixx = wing_mass * b * b / 14
    iyy = sum(i.mass_kg * (i.x - design.cg_x) ** 2 for i in items) + 0.37 * struct * f.length**2 / 12
    if r:
        per = m["lift_system"] / 4
        ixx += 4 * per * r.boom_y**2
        iyy += 4 * per * (0.5 * (r.front_x - r.rear_x)) ** 2
    ixx += (total - wing_mass) * (0.25 * f.width) ** 2
    design.inertia = {"Ixx": ixx, "Iyy": iyy, "Izz": 0.95 * (ixx + iyy)}


def _lift_rotors(design: Design, m: Dict[str, float], cg_x: float) -> None:
    req = design.requirements
    _, rotor_d = hover_power(req, design.mass_kg)
    w = design.wing
    boom_y = max(0.22 * w.span, 0.5 * design.propulsion.prop_diameter + 0.5 * rotor_d + 0.04,
                 0.5 * design.fuselage.width + 0.5 * rotor_d + 0.03)
    arm = 0.5 * rotor_d + 0.6 * w.root_chord + 0.03
    design.lift_rotors = LiftRotors(diameter=rotor_d, boom_y=boom_y, front_x=cg_x + arm, rear_x=cg_x - arm,
                                    boom_diameter=max(0.016, 0.02 * w.span ** 0.5), z=w.z + 0.05)


def build(req: Requirements, model: Optional[Dict[str, float]] = None) -> Tuple[Design, Dict[str, float]]:
    m = _mass_loop(req, model or _first_guess(req))
    design = _geometry(req, m)
    cg = aero.neutral_point(design) + req.static_margin * design.wing.mac
    if req.config == "vtol_quadplane":
        _lift_rotors(design, m, cg)
    balance(design, m, cg)
    return design, m


def size(req: Requirements, passes: int = 3) -> Design:
    """Sizes an aircraft whose *predicted* stall speed and energy use meet the requirements."""
    if req.cruise_speed_ms < 1.2 * req.stall_speed_ms:
        raise SizingError("Cruise speed must be at least 1.2 times the stall speed for a safe margin.")
    airfoils.parse(req.wing_airfoil)
    model = _first_guess(req)
    design, _ = build(req, model)
    for _ in range(passes):
        p = aero.polar(design)
        c0, c1, c2 = p["drag_fit"]
        model = {"cl_max": p["CL_max"], "c0": c0, "c1": c1, "c2": c2}
        design, _ = build(req, model)
    design.notes.insert(0, "Sized so the lifting-line prediction, not a rule of thumb, meets the stall speed "
                           "and endurance requirements.")
    return design


def rebalance(design: Design) -> None:
    """Recomputes derived quantities after the user edits geometry (keeps masses, moves nothing)."""
    total = sum(i.mass_kg for i in design.masses)
    if total > 0:
        design.mass_kg = total
        design.cg_x = sum(i.mass_kg * i.x for i in design.masses) / total
        design.cg_z = sum(i.mass_kg * i.z for i in design.masses) / total
