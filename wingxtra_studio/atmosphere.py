"""International Standard Atmosphere, troposphere only (valid to 11 km)."""

import math

G = 9.80665


def density(altitude_m: float) -> float:
    T = 288.15 - 0.0065 * altitude_m
    return 1.225 * (T / 288.15) ** 4.2559


def viscosity(altitude_m: float) -> float:
    """Dynamic viscosity from Sutherland's law, Pa s."""
    T = 288.15 - 0.0065 * altitude_m
    return 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4)


def reynolds(speed: float, length: float, altitude_m: float) -> float:
    return density(altitude_m) * speed * length / viscosity(altitude_m)


def skin_friction(re: float) -> float:
    """Flat-plate skin friction coefficient, laminar/turbulent blend with transition near Re 5e5."""
    re = max(re, 1e3)
    turbulent = 0.455 / math.log10(re) ** 2.58
    if re < 5e5:
        return max(1.328 / math.sqrt(re), turbulent * 0.6)
    return turbulent
