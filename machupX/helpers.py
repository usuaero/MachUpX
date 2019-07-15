"""A set of useful helper functions for MachUpX to use."""

import os

def _check_filepath(input_filename, correct_ext):
    # Check correct file extension and that file exists
    if correct_ext not in input_filename:
        raise IOError("Input to scene not a JSON object.")
    if not os.path.exists(input_filename):
        raise IOError("Invalid input filepath.")

def _convert_units(in_value, units, system):
    # Converts the given value from the specified units to the default for the system chosen
    to_english_default = {
        "ft" : 1.0,
        "in" : 1/12,
        "m" : 3.28084,
        "cm" : 0.0328084,
        "ft/s" : 1.0,
        "m/s" : 3.28084,
        "mph" : 5280/3600,
        "kph" : 3.28084/3600,
        "kn" : 1.687811
    }

    to_si_default = {
        "m" : 1.0,

    }
    if system is "English":
        return in_value*to_english_default[units]
    else:
        return in_value*to_si_default[units]