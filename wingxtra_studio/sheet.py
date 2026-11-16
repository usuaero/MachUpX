"""Spreadsheet mode: the Wingxtra design-sheet inputs, row for row.

The engineer chooses wing loading, battery pack and airframe weight directly (rather
than letting the sizing loop find them). This module reproduces the sheet's own
estimates and builds the same aircraft as a Design, so the lifting-line solver can
check the sheet's numbers without anything being re-typed into MachUp.
"""

import math
from typing import Dict, Literal, Optional

from pydantic import BaseModel, Field

from . import aero, airfoils, sizing
from .atmosphere import G
from .schema import Design, Fuselage, Propulsion, Requirements, Surface


class SheetInputs(BaseModel):
    name: str = "Spreadsheet design"

    # Mission requirements
    endurance_min: float = Field(120.0, gt=1, le=1440)
    range_km: float = Field(160.0, gt=0, le=5000)
    stall_margin_ms: float = Field(5.0, ge=0, le=30)
    min_airspeed_ms: float = Field(20.0, gt=3, le=60)
    cruise_airspeed_ms: float = Field(23.0, gt=5, le=80)

    # Payload
    payload_kg: float = Field(1.7, ge=0, le=100)
    payload_power_w: float = Field(15.6, ge=0, le=2000)

    # Aircraft
    airframe_kg: float = Field(9.0, gt=0.05, le=300)
    avionics_power_w: float = Field(10.0, ge=0, le=500)
    fuselage_cd: float = Field(0.03, ge=0, le=0.2)
    zero_lift_cd: float = Field(0.017, ge=0, le=0.1)
    air_density: float = Field(1.225, gt=0.5, le=1.3)

    # Propulsion / batteries
    battery_capacity_mah: float = Field(25000.0, gt=100, le=500000)
    energy_density_wh_kg: float = Field(210.0, gt=50, le=400)
    cell_voltage: float = Field(3.7, gt=2.5, le=4.4)
    cells_in_series: int = Field(12, ge=1, le=30)
    battery_count: int = Field(1, ge=1, le=12)
    capacity_used_pct: float = Field(80.0, gt=10, le=100)
    overall_efficiency_pct: float = Field(80.0, gt=10, le=95)

    # Main wing
    wing_loading_g_dm2: float = Field(227.4, gt=10, le=600)
    aspect_ratio: float = Field(12.8, ge=3, le=25)
    taper_ratio: float = Field(0.3, ge=0.2, le=1.0)
    oswald_efficiency: float = Field(0.85, gt=0.4, le=1.0)
    max_lift_coefficient: float = Field(1.12, gt=0.5, le=2.5)
    wing_airfoil: str = "2412"

    # Horizontal stabilizer
    h_volume_ratio: float = Field(0.55, gt=0.1, le=1.2)
    h_moment_arm_m: float = Field(1.2, gt=0.1, le=10)
    h_aspect_ratio: float = Field(6.0, ge=2, le=10)
    h_taper_ratio: float = Field(1.0, ge=0.3, le=1.0)

    # Vertical stabilizer
    v_volume_ratio: float = Field(0.025, gt=0.005, le=0.12)
    v_moment_arm_m: float = Field(1.1, gt=0.1, le=10)
    v_semispan_m: float = Field(0.4, gt=0.03, le=3)
    v_taper_ratio: float = Field(0.88, ge=0.3, le=1.0)

    # V-tail conversion
    tail: Literal["conventional", "v_tail"] = "conventional"
    vtail_span_m: Optional[float] = Field(None, gt=0.05, le=6)  # blank = the sheet's prediction
    vtail_average_chord_m: Optional[float] = Field(None, gt=0.02, le=2)
    vtail_taper_ratio: float = Field(0.8, ge=0.3, le=1.0)

    # Balance (not on the sheet; needed to place the centre of gravity)
    static_margin: float = Field(0.12, ge=0.02, le=0.4)


def estimates(i: SheetInputs) -> Dict:
    """The sheet's own results, using its formulas unchanged."""
    voltage = i.cell_voltage * i.cells_in_series
    battery_kg = i.battery_capacity_mah * i.battery_count * 0.001 * voltage / i.energy_density_wh_kg
    mass = i.payload_kg + i.airframe_kg + battery_kg
    weight = mass * G
    S = mass * 1000 / i.wing_loading_g_dm2 / 100
    semispan = math.sqrt(i.aspect_ratio * S) / 2
    root = 2 * S / (2 * semispan * (1 + i.taper_ratio))
    average_chord = S / (2 * semispan)
    cd0 = i.zero_lift_cd + i.fuselage_cd
    k = math.pi * i.oswald_efficiency * i.aspect_ratio
    loading = weight / S / i.air_density

    V = i.cruise_airspeed_ms
    cl = weight / (0.5 * i.air_density * V * V * S)
    cd = cd0 + cl * cl / k
    ld = cl / cd
    eff = i.overall_efficiency_pct / 100
    power = weight * V / ld / eff
    current = power / voltage
    hours = (i.battery_capacity_mah / 1000 * i.battery_count * i.capacity_used_pct * 0.01) / (
        current + (i.payload_power_w + i.avionics_power_w) / voltage)
    stall = math.sqrt(2 / i.max_lift_coefficient) * math.sqrt(loading)

    h_area = i.h_volume_ratio * average_chord * S / i.h_moment_arm_m
    h_semispan = math.sqrt(i.h_aspect_ratio * h_area) / 2
    h_root = 2 * h_area / (2 * h_semispan * (1 + i.h_taper_ratio))
    v_area = i.v_volume_ratio * S * semispan * 2 / i.v_moment_arm_m
    v_root = 2 * v_area / (i.v_semispan_m * (1 + i.v_taper_ratio))

    vt_area = h_area + v_area
    grow = (vt_area - h_area) / vt_area + 1
    return {
        "voltage": voltage, "battery_kg": battery_kg, "battery_wh": i.battery_capacity_mah * i.battery_count * voltage / 1000,
        "total_mass_kg": mass, "mass_fraction_pct": (i.payload_kg + battery_kg) / mass * 100,
        "wing_area": S, "semispan": semispan, "root_chord": root, "tip_chord": root * i.taper_ratio,
        "average_chord": average_chord,
        "min_drag_airspeed": (1 / (k * cd0)) ** 0.25 * math.sqrt(2 * loading),
        "min_power_airspeed": 2 / (12 * k * cd0) ** 0.25 * math.sqrt(loading),
        "stall_speed": stall, "stall_margin": V - stall,
        "CL": cl, "CD": cd, "LD": ld, "drag_n": weight / ld,
        "propulsive_power_w": power, "cruise_current_a": current,
        "flight_time_min": hours * 60, "range_km": hours * 3600 * V / 1000,
        "h_area": h_area, "h_semispan": h_semispan, "h_root_chord": h_root, "h_tip_chord": h_root * i.h_taper_ratio,
        "v_area": v_area, "v_root_chord": v_root, "v_tip_chord": v_root * i.v_taper_ratio,
        "vtail_required_area": vt_area,
        "vtail_dihedral_deg": math.degrees(math.atan(math.sqrt(v_area / h_area))),
        "vtail_span_prediction": grow * 2 * h_semispan,
        "vtail_chord_prediction": grow * h_area / (2 * h_semispan),
    }


def _altitude_for_density(rho: float) -> float:
    """Inverts the standard atmosphere so the rest of the pipeline sees the sheet's density."""
    return max(0.0, min(6000.0, (1 - (rho / 1.225) ** (1 / 4.2559)) * 288.15 / 0.0065))


def build(i: SheetInputs) -> Design:
    """The sheet's aircraft as a Design, balanced for the requested static margin.

    The sheet does not fix a nose length, so it is lengthened in steps until the battery
    can balance the aircraft from inside the fuselage.
    """
    design = None
    for scale in (1.0, 1.15, 1.3, 1.45, 1.6):
        design = _build(i, scale)
        if not any("Cannot reach" in n for n in design.notes):
            break
    return design


def _build(i: SheetInputs, nose_scale: float) -> Design:
    airfoils.parse(i.wing_airfoil)
    e = estimates(i)
    if e["stall_speed"] >= i.cruise_airspeed_ms:
        raise sizing.SizingError(
            f"The sheet's stall speed ({e['stall_speed']:.1f} m/s) is at or above the cruise airspeed. "
            "Lower the wing loading or raise the cruise airspeed.")
    mass, S, b = e["total_mass_kg"], e["wing_area"], 2 * e["semispan"]
    V = i.cruise_airspeed_ms
    overall = i.overall_efficiency_pct / 100
    drive = 0.9 if overall <= 0.8 else 0.97
    req = Requirements(
        name=i.name, config="conventional", tail=i.tail,
        payload_kg=i.payload_kg, cruise_speed_ms=V,
        stall_speed_ms=max(3.1, V - i.stall_margin_ms), endurance_min=i.endurance_min, range_km=i.range_km,
        cruise_altitude_m=_altitude_for_density(i.air_density),
        aspect_ratio=min(20.0, i.aspect_ratio), taper_ratio=i.taper_ratio, wing_airfoil=i.wing_airfoil,
        static_margin=i.static_margin, battery_wh_per_kg=i.energy_density_wh_kg,
        battery_usable_fraction=i.capacity_used_pct / 100,
        structure_fraction=min(0.6, max(0.11, i.airframe_kg / mass)), avionics_kg=0.0,
        avionics_power_w=min(500.0, i.payload_power_w + i.avionics_power_w),
        prop_efficiency=min(0.9, overall / drive), drive_efficiency=drive,
    )

    section = airfoils.section_properties(i.wing_airfoil)
    slope = section["CLa"] / (1 + section["CLa"] / (math.pi * i.aspect_ratio))
    incidence = max(0.0, min(6.0, math.degrees(e["CL"] / slope + section["aL0"])))
    wing = Surface(span=b, root_chord=e["root_chord"], tip_chord=e["tip_chord"], dihedral_deg=3.0,
                   incidence_deg=incidence, washout_deg=-2.0, airfoil=i.wing_airfoil,
                   control_chord_fraction=0.25, control_span_start=0.55, control_span_end=0.95)

    arm = max(i.h_moment_arm_m, i.v_moment_arm_m)
    width = min(0.45, max(0.08, 0.5 * wing.mac))
    height = 1.1 * width
    nose_x = 0.25 * wing.root_chord + nose_scale * max(0.16 * b, 1.3 * wing.root_chord)
    wing.z = -0.4 * height
    fuselage = Fuselage(length=nose_x + arm + 0.8 * e["h_root_chord"], width=width, height=height, nose_x=nose_x)

    htail = Surface(span=2 * e["h_semispan"], root_chord=e["h_root_chord"], tip_chord=e["h_tip_chord"],
                    airfoil="0010", x=-i.h_moment_arm_m, z=-0.1 * height, control_chord_fraction=0.35)
    vtail = Surface(span=i.v_semispan_m, root_chord=e["v_root_chord"], tip_chord=e["v_tip_chord"], airfoil="0010",
                    x=-i.v_moment_arm_m, z=-0.5 * height, sweep_deg=10.0, control_chord_fraction=0.35)
    tail_type = "conventional"
    if i.tail == "v_tail":
        span = i.vtail_span_m or e["vtail_span_prediction"]
        chord = i.vtail_average_chord_m or e["vtail_chord_prediction"]
        htail = sizing.v_tail(e["h_area"], e["v_area"], i.h_moment_arm_m, z=-0.3 * height, span=span,
                              taper=i.vtail_taper_ratio, average_chord=chord)
        vtail, tail_type = None, "v_tail"

    # The sheet has no motor: size one for cruise plus a 3 m/s climb, as the sizing loop does.
    weight = mass * G
    shaft = max((weight / e["LD"] * V + weight * req.climb_rate_ms) / req.prop_efficiency,
                1.5 * weight / e["LD"] * V / req.prop_efficiency)
    cruise_system = min(0.5 * i.airframe_kg, 0.05 + 0.45 * shaft / 1000)

    design = Design(
        requirements=req, mass_kg=mass, wing=wing, tail_type=tail_type, htail=htail, vtail=vtail, fuselage=fuselage,
        propulsion=Propulsion(layout="tractor", prop_diameter=0.43 * (shaft / 1000) ** 0.25, motor_power_w=shaft,
                              prop_x=nose_x + 0.02),
        battery_wh=e["battery_wh"], body_cd_override=i.fuselage_cd, source="spreadsheet",
        sheet_inputs=i.model_dump(),
    )
    cg = aero.neutral_point(design) + i.static_margin * wing.mac
    sizing.balance(design, {"structure": i.airframe_kg - cruise_system, "cruise_system": cruise_system,
                            "battery": e["battery_kg"], "lift_system": 0.0}, cg)
    design.notes.insert(0, "Built from your spreadsheet inputs. The airframe weight is split into structure and "
                           "an estimated motor, ESC and propeller; the motor is sized for a 3 m/s climb.")
    return design


def comparison(i: SheetInputs, analysis: Dict) -> list:
    """Sheet estimate against solver prediction, quantity by quantity."""
    e = estimates(i)
    perf, trim, polar = analysis["performance"], analysis["trim"], analysis["polar"]
    cruise = perf["cruise"]
    rows = [
        ("Stall speed", "m/s", e["stall_speed"], perf["stall_speed_ms"]),
        ("Maximum lift coefficient", "", i.max_lift_coefficient, polar["CL_max"]),
        ("Cruise lift coefficient", "", e["CL"], trim["CL"]),
        ("Cruise drag coefficient", "", e["CD"], trim["CD"]),
        ("Cruise lift-to-drag ratio", "", e["LD"], trim["LD"]),
        ("Best lift-to-drag ratio", "", 0.5 * math.sqrt(math.pi * i.oswald_efficiency * i.aspect_ratio
                                                          / (i.zero_lift_cd + i.fuselage_cd)), polar["LD_max"]),
        ("Cruise electrical power", "W", e["propulsive_power_w"] + i.payload_power_w + i.avionics_power_w,
         cruise["electrical_power_w"]),
        ("Flight time", "min", e["flight_time_min"], cruise["endurance_min"]),
        ("Range", "km", e["range_km"], cruise["range_km"]),
    ]
    return [{"quantity": q, "unit": u, "sheet": a, "solver": b, "difference_pct": (b - a) / a * 100 if a else None}
            for q, u, a, b in rows]
