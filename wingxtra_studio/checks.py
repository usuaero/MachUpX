"""Design review: turns the numbers into pass / warn / fail verdicts with advice.

Thresholds follow common small-UAV practice and the MIL-F-8785C Level 1 flying
qualities boundaries where one exists.
"""

import math
from typing import Dict, List, Optional

from .schema import Design


def _check(title: str, status: str, value: str, target: str, advice: str = "") -> Dict:
    return {"title": title, "status": status, "value": value, "target": target,
            "advice": advice if status != "pass" else ""}


def _band(x: float, ok: tuple, warn: tuple) -> str:
    if ok[0] <= x <= ok[1]:
        return "pass"
    return "warn" if warn[0] <= x <= warn[1] else "fail"


def review(design: Design, polar: Dict, trim: Dict, perf: Dict, dyn: Dict) -> List[Dict]:
    req = design.requirements
    out: List[Dict] = []
    stab = trim["stability"]
    modes = dyn["modes"]
    flying_wing = design.htail is None

    # ---- mission
    vs = perf["stall_speed_ms"]
    out.append(_check("Stall speed", "pass" if vs <= req.stall_speed_ms * 1.02 else "warn" if vs <= req.stall_speed_ms * 1.1 else "fail",
                      f"{vs:.1f} m/s", f"at most {req.stall_speed_ms:.1f} m/s",
                      "Increase wing area or choose an airfoil with more camber."))
    e = perf["cruise"]["endurance_min"]
    out.append(_check("Cruise endurance", "pass" if e >= req.endurance_min * 0.98 else "warn" if e >= req.endurance_min * 0.9 else "fail",
                      f"{e:.0f} min", f"at least {req.endurance_min:.0f} min",
                      "Add battery, reduce drag, or cruise nearer the best-endurance speed."))
    margin = req.cruise_speed_ms / vs
    out.append(_check("Cruise stall margin", _band(margin, (1.3, 99), (1.2, 99)), f"{margin:.2f} x stall speed", "at least 1.3",
                      "Cruising this close to stall leaves little margin for gusts and turns."))
    climb = perf["cruise"]["climb_rate_ms"]
    out.append(_check("Climb rate at cruise speed", "pass" if climb >= req.climb_rate_ms * 0.95 else "warn" if climb >= 1.5 else "fail",
                      f"{climb:.1f} m/s", f"at least {req.climb_rate_ms:.1f} m/s",
                      "Fit a more powerful cruise motor."))
    vmax = perf["max_speed_ms"]
    if vmax is None:
        out.append(_check("Level flight", "fail", "not sustainable", "power available exceeds power required",
                          "The cruise motor cannot hold level flight at any speed. Increase motor power."))

    # ---- stability
    sm = trim["static_margin"]
    out.append(_check("Static margin", _band(sm, (0.05, 0.20), (0.02, 0.30)), f"{sm * 100:.1f}% MAC", "5% to 20% MAC",
                      "Negative or tiny margins are unstable in pitch; move the centre of gravity forward. "
                      "Very large margins cost trim drag and elevator authority; move it aft."))
    cnb = stab["Cn,b"]
    out.append(_check("Directional stability", _band(cnb, (0.04, 9), (0.015, 9)), f"Cn_beta = {cnb:.3f} /rad", "at least 0.04 /rad",
                      "Enlarge the fin or winglets, or lengthen the tail arm."))
    clb = stab["Cl,b"]
    out.append(_check("Dihedral effect", _band(-clb, (0.02, 0.25), (0.0, 0.4)), f"Cl_beta = {clb:.3f} /rad", "-0.02 to -0.25 /rad",
                      "Add dihedral if too weak; reduce dihedral or sweep if too strong (Dutch roll worsens)."))

    # ---- control
    de = trim["elevator"]
    out.append(_check("Elevator to trim at cruise", _band(abs(de), (0, 6), (0, 12)), f"{de:+.1f} deg", "within 6 deg",
                      "Adjust wing or tail incidence so the aircraft trims with a nearly neutral elevator; "
                      "a deflected elevator in cruise costs drag."))
    slow = trim.get("near_stall")
    if slow:
        out.append(_check("Elevator to hold near-stall lift", _band(abs(slow["elevator"]), (0, 15), (0, 22)),
                          f"{slow['elevator']:+.1f} deg", "within 15 deg",
                          "The elevator is close to running out of authority at low speed. Enlarge it, or "
                          "reduce the static margin."))
    roll = dyn["roll_rate_dps_at_full_aileron"]
    out.append(_check("Roll authority", _band(roll, (60, 1e9), (35, 1e9)), f"{roll:.0f} deg/s at 20 deg aileron", "at least 60 deg/s",
                      "Enlarge the ailerons or move them further outboard."))

    # ---- stall behaviour
    where = polar["stall_span_fraction"]
    out.append(_check("Stall begins", _band(where, (0, 0.6), (0, 0.75)), f"at {where * 100:.0f}% of semispan", "inboard of 60%",
                      "The outer wing stalls first, which causes a wing drop and loss of aileron control. "
                      "Add washout or reduce taper."))

    # ---- flying qualities
    sp = modes.get("short_period")
    if sp:
        if sp.get("oscillatory"):
            z = sp["damping_ratio"]
            out.append(_check("Short-period damping", _band(z, (0.35, 1.3), (0.25, 2.0)), f"zeta = {z:.2f}", "0.35 to 1.3",
                              "Enlarge the horizontal tail or lengthen the tail arm to add pitch damping."))
        else:
            out.append(_check("Short-period mode", "pass" if sp["stable"] else "fail",
                              "non-oscillatory, " + ("stable" if sp["stable"] else "unstable"), "stable",
                              "The aircraft diverges in pitch. Move the centre of gravity forward."))
    ph = modes.get("phugoid")
    if ph and ph.get("oscillatory"):
        z = ph["damping_ratio"]
        out.append(_check("Phugoid damping", _band(z, (0.04, 9), (0.0, 9)), f"zeta = {z:.3f}, period {ph['period_s']:.0f} s", "at least 0.04",
                          "A lightly damped phugoid is slow enough for an autopilot to suppress, but a negative "
                          "value needs attention."))
    dr = modes.get("dutch_roll")
    if dr:
        z, wn = dr["damping_ratio"], dr["natural_frequency_rad_s"]
        out.append(_check("Dutch-roll damping", "pass" if z >= 0.08 and z * wn >= 0.15 else "warn" if z > 0.02 else "fail",
                          f"zeta = {z:.2f}, {wn:.1f} rad/s", "zeta at least 0.08",
                          "Enlarge the fin for more yaw damping, or reduce dihedral."))
    rl = modes.get("roll")
    if rl and rl.get("time_constant_s"):
        tau = rl["time_constant_s"]
        out.append(_check("Roll response", _band(tau, (0, 1.0), (0, 1.4)), f"time constant {tau:.2f} s", "under 1.0 s",
                          "Roll response is sluggish; reduce roll inertia or span."))
    sp_ = modes.get("spiral")
    if sp_:
        if sp_["stable"]:
            out.append(_check("Spiral mode", "pass", "stable", "stable or slow to diverge"))
        else:
            t2 = sp_.get("time_to_half_or_double_s", 1e9)
            out.append(_check("Spiral mode", _band(t2, (12, 1e12), (4, 1e12)), f"doubles in {t2:.0f} s", "at least 12 s",
                              "A fast spiral divergence rolls the aircraft into a tightening turn. Add dihedral "
                              "or reduce fin area."))

    # ---- layout
    f = design.fuselage
    battery = next((i for i in design.masses if i.name == "Battery"), None)
    if battery and "Cannot reach" in battery.note:
        out.append(_check("Balance", "fail", "battery cannot balance the aircraft", "battery fits within the bay",
                          battery.note + ". Move the wing, lengthen the nose, or add ballast."))
    elif battery:
        out.append(_check("Balance", "pass", f"battery {(f.nose_x - battery.x) * 1000:.0f} mm aft of nose", "battery fits within the bay"))

    ar = design.wing.span**2 / design.wing.area
    if ar < 4:
        out.append(_check("Method validity", "warn", f"aspect ratio {ar:.1f}", "at least 4",
                          "Lifting-line theory loses accuracy below aspect ratio 4. Treat results as indicative."))

    r = design.lift_rotors
    if r and perf["hover"]:
        h = perf["hover"]
        out.append(_check("Hover endurance", "pass" if h["full_battery_hover_min"] >= 2 * req.hover_time_min else "warn",
                          f"{h['full_battery_hover_min']:.1f} min on a full battery",
                          f"at least {2 * req.hover_time_min:.0f} min (twice the planned hover)",
                          "Little reserve for an aborted transition. Add battery or lower disk loading."))
        clear = r.boom_y - 0.5 * r.diameter - 0.5 * max(f.width, design.propulsion.prop_diameter)
        out.append(_check("Rotor clearance", "pass" if clear >= 0.02 else "fail", f"{clear * 1000:.0f} mm to fuselage and propeller disc",
                          "at least 20 mm", "Move the rotor booms further outboard."))
    return out


def summary(checks: List[Dict]) -> Dict[str, int]:
    return {s: sum(1 for c in checks if c["status"] == s) for s in ("pass", "warn", "fail")}
