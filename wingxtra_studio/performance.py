"""Point performance from the predicted drag polar: power, endurance, range, climb, hover."""

import math
from typing import Dict

import numpy as np

from .atmosphere import G, density
from .schema import Design
from .sizing import FIGURE_OF_MERIT, HOVER_THRUST_MARGIN


def performance(design: Design, polar: Dict, trim: Dict) -> Dict:
    req = design.requirements
    rho = density(req.cruise_altitude_m)
    W, S = design.mass_kg * G, design.wing.area
    c0, c1, c2 = polar["drag_fit"]
    prop = design.propulsion

    v_stall = math.sqrt(2 * W / (rho * S * polar["CL_max"]))

    hover = None
    hover_energy = 0.0
    if design.lift_rotors:
        r = design.lift_rotors
        disk = r.count * math.pi * r.diameter**2 / 4
        ideal = W * math.sqrt(W / disk / (2 * rho))
        elec = ideal / FIGURE_OF_MERIT / req.drive_efficiency
        hover_energy = elec * req.hover_time_min * 60
        hover = {
            "power_w": elec,
            "disk_loading": W / disk,
            "thrust_to_weight": HOVER_THRUST_MARGIN,
            "thrust_per_rotor_n": W * HOVER_THRUST_MARGIN / r.count,
            "energy_wh": hover_energy / 3600,
            "full_battery_hover_min": design.battery_wh * req.battery_usable_fraction * 60 / elec,
        }

    usable_j = design.battery_wh * 3600 * req.battery_usable_fraction
    cruise_j = max(0.0, usable_j - hover_energy)

    def at(v: float) -> Dict[str, float]:
        q = 0.5 * rho * v * v
        cl = W / (q * S)
        cd = c0 + c1 * cl + c2 * cl * cl
        drag = q * S * cd
        shaft = drag * v / req.prop_efficiency
        elec = shaft / req.drive_efficiency + req.avionics_power_w
        endurance = cruise_j / elec
        return {
            "speed": v, "CL": cl, "CD": cd, "LD": cl / cd, "drag_n": drag,
            "shaft_power_w": shaft, "electrical_power_w": elec,
            "endurance_min": endurance / 60, "range_km": v * endurance / 1000,
            "climb_rate_ms": (prop.motor_power_w * req.prop_efficiency - drag * v) / W,
            "sink_rate_ms": drag * v / W,
        }

    speeds = np.linspace(1.03 * v_stall, max(2.6 * v_stall, 1.6 * req.cruise_speed_ms), 70)
    curve = [at(float(v)) for v in speeds]
    flyable = [p for p in curve if p["shaft_power_w"] <= prop.motor_power_w]
    cruise = at(req.cruise_speed_ms)
    best_e = max(curve, key=lambda p: p["endurance_min"])
    best_r = max(curve, key=lambda p: p["range_km"])
    best_climb = max(curve, key=lambda p: p["climb_rate_ms"])

    lift_slope = trim["stability"]["CL,a"]
    return {
        "stall_speed_ms": v_stall,
        "cruise": cruise,
        "best_endurance": best_e,
        "best_range": best_r,
        "max_speed_ms": flyable[-1]["speed"] if flyable else None,
        "max_climb_rate_ms": best_climb["climb_rate_ms"],
        "best_climb_speed_ms": best_climb["speed"],
        "min_sink_ms": min(p["sink_rate_ms"] for p in curve),
        "glide_ratio": max(p["LD"] for p in curve),
        "wing_loading_kg_m2": design.mass_kg / S,
        "power_loading_w_kg": prop.motor_power_w / design.mass_kg,
        "usable_energy_wh": usable_j / 3600,
        "gust_load_factor_per_ms": rho * req.cruise_speed_ms * lift_slope * S / (2 * W),
        "hover": hover,
        "curve": curve,
    }
