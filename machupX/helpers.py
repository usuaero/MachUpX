"""A set of useful helper functions for MachUpX to use."""

import os
import numpy as np
import copy
import json


def check_filepath(input_filename, correct_ext):
    # Check correct file extension and that file exists
    if correct_ext not in input_filename:
        raise IOError("File {0} has the wrong extension. Expected a {1} file.".format(input_filename, correct_ext))
    if not os.path.exists(input_filename):
        raise IOError("Cannot find file {0}.".format(input_filename))


def convert_units(in_value, units, system):
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


def import_value(key, dict_of_vals, system, default_value):
    # Imports value from a dictionary. Handles importing arrays from files and 
    # unit conversions. If default_value is -1, then this value must be 
    # specified in the input (i.e. an error is thrown if -1 is returned).

    val = dict_of_vals.get(key, default_value)
    
    if val is None:
        raise IOError("Key '{0}' is not optional. Please specify.".format(key))
    is_array = False

    if isinstance(val, float): # Float without units
        return_value = val

    elif isinstance(val, int): # Integer values should be converted to floats
        return_value = float(val)

    elif isinstance(val, str) and ".csv" in val: # Filepath containing array
        check_filepath(val, ".csv")
        with open(val, 'r') as array_file:
            val = np.genfromtxt(array_file, delimiter=',', dtype=None, encoding='utf-8')
            is_array = True
            
    elif isinstance(val, str): # Simply a string value
        return_value = val

    elif isinstance(val, list):
        if any(isinstance(row, list) for row in val): # Array
            is_array = True

        elif val[0] == "elliptic": # User wants an elliptic chord distribution
            if len(val) == 3: # Unit specified
                root_chord = convert_units(val[1], val[2], system)
            else:
                root_chord = val[1]
            return_value = ("elliptic", root_chord)
        
        elif isinstance(val[-1], str): # Float or vector with units
            converted_val = vectorized_convert_units(val[:-1], val[-1], system)

            try:
                return_value = converted_val.item() # Float
            except ValueError:
                return_value = converted_val # Vector or quaternion

        elif len(val) == 3 or len(val) == 4: # Vector or quaternion without units
            return_value = np.asarray(val)

        else:
            raise ValueError("Did not recognize value format {0}.".format(val))

    elif callable(val):
        return_value = val

    else:
        raise ValueError("Did not recognize value format {0}.".format(val))

    if is_array:
        if isinstance(val[-1][0], str): # Array with units
            val = np.asarray(val)
            units = val[-1,:]
            data = val[:-1,:].astype(float)
            return_value = vectorized_convert_units(data, units, system)

        else: # Array without units
            #TODO: Allow this to handle arrays specifying a distribution of airfoils
            return_value = np.asarray(val)

    return return_value


vectorized_convert_units = np.vectorize(convert_units)


def quat_trans(q, v):
    # Transforms the vector v from the global frame to a frame having an orientation described by q.
    v_T = np.transpose(v)
    T = np.zeros((4,*v_T.shape[1:]))
    v_trans = np.zeros_like(v_T)

    T[0] = -v_T[0]*q[1] - v_T[1]*q[2] - v_T[2]*q[3]
    T[1] =  v_T[0]*q[0] + v_T[1]*q[3] - v_T[2]*q[2]
    T[2] = -v_T[0]*q[3] + v_T[1]*q[0] + v_T[2]*q[1]
    T[3] =  v_T[0]*q[2] - v_T[1]*q[1] + v_T[2]*q[0]

    v_trans[0] = q[0]*T[1] - q[1]*T[0] - q[2]*T[3] + q[3]*T[2]
    v_trans[1] = q[0]*T[2] + q[1]*T[3] - q[2]*T[0] - q[3]*T[1]
    v_trans[2] = q[0]*T[3] - q[1]*T[2] + q[2]*T[1] - q[3]*T[0]

    return np.transpose(v_trans)


def quat_inv_trans(q, v):
    # Transforms the vector v from a frame having an orientation described by q to the global frame.
    v_T = np.transpose(v)
    T = np.zeros((4,*v_T.shape[1:]))
    v_trans = np.zeros_like(v_T)

    T[0] =  v_T[0]*q[1] + v_T[1]*q[2] + v_T[2]*q[3]
    T[1] =  v_T[0]*q[0] - v_T[1]*q[3] + v_T[2]*q[2]
    T[2] =  v_T[0]*q[3] + v_T[1]*q[0] - v_T[2]*q[1]
    T[3] = -v_T[0]*q[2] + v_T[1]*q[1] + v_T[2]*q[0]

    v_trans[0] = q[0]*T[1] + q[1]*T[0] + q[2]*T[3] - q[3]*T[2]
    v_trans[1] = q[0]*T[2] - q[1]*T[3] + q[2]*T[0] + q[3]*T[1]
    v_trans[2] = q[0]*T[3] + q[1]*T[2] - q[2]*T[1] + q[3]*T[0]
    
    return np.transpose(v_trans)


def quat_mult(quat0, quat1):
    # Multiplies the two quaternions together and returns the resulting quaternion. Can handle being passed a
    # vector

    # Check for vectors
    if quat0.size[0] == 3:
        q0 = np.concatenate((np.zeros(1), quat0))
    else:
        q0 = copy.copy(quat0)

    if quat1.size[0] == 3:
        q1 = np.concatenate((np.zeros(1), quat1))
    else:
        q1 = copy.copy(quat1)

    # Multiply
    q = np.zeros(4)
    q[0] = q0[0]*q1[0]-q0[1]*q1[1]-q0[2]*q1[2]-q0[3]*q1[3]
    q[1] = q0[0]*q1[1]+q0[1]*q1[0]+q0[2]*q1[3]-q0[3]*q1[2]
    q[2] = q0[0]*q1[2]-q0[1]*q1[3]+q0[2]*q1[0]+q0[3]*q1[1]
    q[3] = q0[0]*q1[3]+q0[1]*q1[2]-q0[2]*q1[1]+q0[3]*q1[0]

    return q


def euler_to_quat(E):
    # Converts the Euler angles phi, theta, psi to an orientation quaternion
    # Phillips Mech. of Flight 11.7.8
    q = np.zeros(4)

    # Trigonometric values
    C_phi = np.cos(E[0]/2)
    S_phi = np.sin(E[0]/2)
    C_theta = np.cos(E[1]/2)
    S_theta = np.sin(E[1]/2)
    C_psi = np.cos(E[2]/2)
    S_psi = np.sin(E[2]/2)

    # Create quaternion
    q[0] = C_phi*C_theta*C_psi + S_phi*S_theta*S_psi
    q[1] = S_phi*C_theta*C_psi - C_phi*S_theta*S_psi
    q[2] = C_phi*S_theta*C_psi + S_phi*C_theta*S_psi
    q[3] = C_phi*C_theta*S_psi - S_phi*S_theta*C_psi

    return q


def quat_to_euler(q):
    # Converts an orientation quaternion to Euler angles
    # Phillips Mech. of Flight 11.7.11
    E = np.zeros(3)
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    quantity = q0*q2-q1*q3

    if quantity != 0.5 and quantity != -0.5:
        q02 = q0*q0
        q12 = q1*q1
        q22 = q2*q2
        q32 = q3*q3
        E[0] = np.arctan2(2*(q0*q1+q2*q3), q02+q32-q12-q22)
        E[1] = np.arcsin(2*(q0*q2-q1*q3))
        E[2] = np.arctan2(2*(q0*q3+q1*q2), q02+q12-q22-q32)
    else:
        E[0] = 2*np.arcsin(q1/np.cos(np.pi/4))
        E[1] = np.pi*quantity
        E[2] = 0.0

    return E


def quat_conj(q):
    return [q[0], -q[1], -q[2], -q[3]]


def parse_input(mux_input):
    """Takes an input to MachUpX and converts it to a scene dictionary and lists of airplanes, states, and control states.
    This is really just used to simplify things in the unit tests.

    Parameters
    ----------
    mux_input : dict or str
        Scene input to MachUpX

    Returns
    -------
    dict
        Scene input stripped of airplanes

    list
        Airplane names

    list
        Airplane dictionaries

    list
        State dictionaries

    list
        Control state dictionaries
    """

    # Load input
    if isinstance(mux_input, str):
        with open(mux_input, 'r') as input_handle:
            scene_dict = json.load(input_handle)

    else:
        scene_dict = copy.deepcopy(mux_input)

    # Get aircraft
    airplane_names = []
    airplanes = []
    states = []
    control_states = []
    for key, value in scene_dict["scene"].pop("aircraft", {}).items():
        airplane_names.append(key)

        # Load airplane object
        with open(value["file"], 'r') as airplane_file:
            airplanes.append(json.load(airplane_file))

        states.append(value.get("state", {}))
        control_states.append(value.get("control_state", {}))

    if len(airplane_names) == 1:
        return scene_dict, airplane_names[0], airplanes[0], states[0], control_states[0]
    else:
        return scene_dict, airplane_names, airplanes, states, control_states