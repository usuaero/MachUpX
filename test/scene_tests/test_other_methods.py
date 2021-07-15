# Tests various other scene methods

import machupX as MX
import numpy as np
import json
import subprocess as sp

input_file = "test/input_for_testing.json"


def test_target_CL():

    # Alter input
    with open(input_file, 'r') as input_file_handle:
        input_dict = json.load(input_file_handle)

    input_dict["scene"]["aircraft"]["test_plane"]["control_state"] = {
        "elevator" : 0.0,
        "rudder" : 0.0,
        "aileron" : 0.0
    }

    input_dict["scene"]["aircraft"]["test_plane"]["state"] = {
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 0.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    alpha = scene.target_CL(CL=0.5)
    print(alpha)
    assert abs(alpha-4.531085683208453)<1e-10


def test_pitch_trim():

    # Alter input
    with open(input_file, 'r') as input_file_handle:
        input_dict = json.load(input_file_handle)

    input_dict["scene"]["aircraft"]["test_plane"]["control_state"] = {
        "elevator" : 0.0,
        "rudder" : 0.0,
        "aileron" : 0.0
    }

    input_dict["scene"]["aircraft"]["test_plane"]["state"] = {
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 0.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    trim = scene.pitch_trim()
    print(trim)
    assert abs(trim["test_plane"]["alpha"]-12.27375383887546)<1e-10
    assert abs(trim["test_plane"]["elevator"]+10.303072276682208)<1e-10


def test_pitch_trim_with_finite_moment():

    # Alter input
    with open(input_file, 'r') as input_file_handle:
        input_dict = json.load(input_file_handle)

    input_dict["scene"]["aircraft"]["test_plane"]["control_state"] = {
        "elevator" : 0.0,
        "rudder" : 0.0,
        "aileron" : 0.0
    }

    input_dict["scene"]["aircraft"]["test_plane"]["state"] = {
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 0.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    trim = scene.pitch_trim(CL=0.1, Cm=0.2)
    print(trim)
    assert abs(trim["test_plane"]["alpha"]-1.9195151950855374)<1e-10
    assert abs(trim["test_plane"]["elevator"]+3.807322736307231)<1e-10


def test_pitch_trim_orientation():

    # Alter input
    with open(input_file, 'r') as input_file_handle:
        input_dict = json.load(input_file_handle)

    input_dict["scene"]["aircraft"]["test_plane"]["control_state"] = {
        "elevator" : 0.0,
        "rudder" : 0.0,
        "aileron" : 0.0
    }

    input_dict["scene"]["aircraft"]["test_plane"]["state"] = {
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 0.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    trim = scene.pitch_trim_using_orientation(CL=0.1, Cm=0.2)
    print(trim)
    assert abs(trim[0]["orientation"][0]-0.9999997533435244)<1e-10
    assert abs(trim[0]["orientation"][1])<1e-10
    assert abs(trim[0]["orientation"][2]+0.0007023623640411208)<1e-10
    assert abs(trim[0]["orientation"][3])<1e-10
    assert abs(trim[1]["elevator"]+3.8073227363076954)<1e-10