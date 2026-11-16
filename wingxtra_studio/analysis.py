"""One call that runs every analysis on a design."""

from typing import Dict

from . import aero, checks, dynamics, performance
from .schema import Design


def analyze(design: Design) -> Dict:
    polar = aero.polar(design)
    trim = aero.trim_and_derivatives(design, polar["CL_max"])
    perf = performance.performance(design, polar, trim)
    dyn = dynamics.dynamics(design, trim)
    review = checks.review(design, polar, trim, perf, dyn)
    w = design.wing
    return {
        "geometry": {
            "wing_area": w.area, "span": w.span, "mac": w.mac, "aspect_ratio": w.span**2 / w.area,
            "length": design.fuselage.length, "mass_kg": design.mass_kg, "battery_wh": design.battery_wh,
        },
        "polar": polar,
        "trim": trim,
        "drag_buildup": aero.parasite_buildup(design),
        "performance": perf,
        "dynamics": dyn,
        "checks": review,
        "summary": checks.summary(review),
    }
