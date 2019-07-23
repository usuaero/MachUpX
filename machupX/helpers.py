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
    if units == "-":
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
            return in_value*to_english_default[units.strip(' \t\r\n')]
        else:
            return in_value*to_si_default[units.strip(' \t\r\n')]
    except KeyError:
        raise IOError("Improper units specified; {0} is not an allowable unit definition.".format(units))

_vectorized_convert_units = np.vectorize(_convert_units)

def _import_value(key, dict_of_vals, system, default_value):
    # Imports value from a dictionary. Handles importing arrays from files and 
    # unit conversions. If default_value is -1, then this value must be 
    # specified in the input (i.e. an error is thrown if -1 is returned).

    val = dict_of_vals.get(key, default_value)
    
    if val == -1:
        raise IOError("Key {0} is not optional. Please specify.".format(key))
    is_array = False

    if isinstance(val, float): # Float without units
        return_value = val

    elif isinstance(val, int): # Integer values should be converted to floats
        return_value = float(val)

    elif isinstance(val, str) and ".csv" in val: # Filepath containing array
        _check_filepath(val, ".csv")
        with open(val, 'r') as array_file:
            val = np.genfromtxt(array_file, delimiter=',', dtype=None, encoding='utf-8')
            is_array = True
            
    elif isinstance(val, str): # Simply a string value
        return_value = val

    elif isinstance(val, list):
        if any(isinstance(row, list) for row in val): # Array
            is_array = True
        
        elif isinstance(val[-1], str): # Float or vector with units
            converted_val = _vectorized_convert_units(val[:-1], val[-1], system)

            try:
                return_value = converted_val.item() # Float
            except ValueError:
                return_value = converted_val # Vector or quaternion

        elif len(val) == 3 or len(val) == 4: # Vector or quaternion without units
            return_value = np.asarray(val)

        else:
            raise ValueError("Did not recognize value format {0}.".format(val))

    else:
        raise ValueError("Did not recognize value format {0}.".format(val))

    if is_array:
        if isinstance(val[-1][0], str): # Array with units
            val = np.asarray(val)
            units = val[-1,:]
            data = val[:-1,:].astype(float)
            return_value = _vectorized_convert_units(data, units, system)

        else: # Array without units
            #TODO: Allow this to handle arrays specifying a distribution of airfoils
            return_value = np.asarray(val)

    return return_value


def _quaternion_transform(q, v):
    # Transforms the vector v from the global frame to a frame having an orientation described by q.
    T = np.zeros(4)
    T[0] = -v[0]*q[1] - v[1]*q[2] - v[2]*q[3]
    T[1] = v[0]*q[0] + v[1]*q[3] - v[2]*q[2]
    T[2] = -v[0]*q[3] + v[1]*q[0] + v[2]*q[1]
    T[3] = v[0]*q[2] - v[1]*q[1] + v[2]*q[0]

    v_trans = np.zeros(3)
    v_trans[0] = q[0]*T[1] - q[1]*T[0] - q[2]*T[3] + q[3]*T[2]
    v_trans[1] = q[0]*T[2] + q[1]*T[3] - q[2]*T[0] - q[3]*T[1]
    v_trans[2] = q[0]*T[3] - q[1]*T[2] + q[2]*T[1] - q[3]*T[0]

    return v_trans


def _quaternion_inverse_transform(q, v):
    # Transforms the vector v from a frame having an orientation described by q to the global frame.
    T = np.zeros(4)
    T[0] = v[0]*q[1] + v[1]*q[2] + v[2]*q[3]
    T[1] = v[0]*q[0] - v[1]*q[3] + v[2]*q[2]
    T[2] = v[0]*q[3] + v[1]*q[0] - v[2]*q[1]
    T[3] = -v[0]*q[2] + v[1]*q[1] + v[2]*q[0]

    v_trans = np.zeros(3)
    v_trans[0] = q[0]*T[1] + q[1]*T[0] + q[2]*T[3] - q[3]*T[2]
    v_trans[1] = q[0]*T[2] - q[1]*T[3] + q[2]*T[0] + q[3]*T[1]
    v_trans[2] = q[0]*T[3] + q[1]*T[2] - q[2]*T[1] + q[3]*T[0]

    return v_trans