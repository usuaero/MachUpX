"""A set of useful helper functions for MachUpX to use."""

import os
import numpy as np

def _check_filepath(input_filename, correct_ext):
    # Check correct file extension and that file exists
    if correct_ext not in input_filename:
        raise IOError("File {0} has the wrong extension. Expected a {1} file.".format(input_filename, correct_ext))
    if not os.path.exists(input_filename):
        raise IOError("Cannot find file {0}.".format(input_filename))

def _convert_units(in_value, units, system):
    # Converts the given value from the specified units to the default for the system chosen
    if units is "-":
        return in_value

    to_english_default = {
        "ft" : 1.0,
        "in" : 0.083333333,
        "m" : 3.28084,
        "cm" : 0.0328084,
        "ft/s" : 1.0,
        "m/s" : 3.28084,
        "mph" : 1.466666666,
        "kph" : 0.9113444,
        "kn" : 1.687811,
        "ft^2" : 1.0,
        "m^2" : 10.76391,
        "slug/ft^3" : 1.0,
        "kg/m^3" : 0.0019403203,
        "lbf" : 1.0,
        "N" : 0.22480894244319,
        "deg" : 1.0,
        "rad" : 57.29578,
        "deg/s" : 0.01745329,
        "rad/s" : 1.0
    }

    to_si_default = {
        "ft" : 0.3048,
        "in" : 0.0254,
        "m" : 1.0,
        "cm" : 0.01,
        "ft/s" : 0.3048,
        "m/s" : 1.0,
        "mph" : 0.44704,
        "kph" : 0.277777777,
        "kn" : 0.514444444,
        "ft^2" : 0.09290304,
        "m^2" : 1.0,
        "slug/ft^3" : 515.378819,
        "kg/m^3" : 1.0,
        "lbf" : 4.4482216,
        "N" : 1.0,
        "deg" : 1.0,
        "rad" : 57.29578,
        "deg/s" : 0.01745329,
        "rad/s" : 1.0
    }
    try:
        if system == "English":
            return in_value*to_english_default[units]
        else:
            return in_value*to_si_default[units]
    except KeyError:
        raise IOError("Improper units specified; {0} is not an allowable unit definition.".format(units))

_vectorized_convert_units = np.vectorize(_convert_units)