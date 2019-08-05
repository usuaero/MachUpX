# Tests that the NLL algorithm is correctly solved

import machupX as MX
import numpy as np
import json
import subprocess as sp

input_file = "test/NLL_tests/input_for_NLL_testing.json"

def test_linear_NLL():
    # Tests the NLL algorithm is correctly solved

    # Alter input
    with open(input_file, 'r') as input_file_handle:
        input_dict = json.load(input_file_handle)

    input_dict["solver"]["type"] = "linear"

    input_dict["scene"]["aircraft"]["test_plane"]["control_state"] = {
        "elevator" : 0.0,
        "rudder" : 0.0,
        "aileron" : 0.0
    }

    input_dict["scene"]["aircraft"]["test_plane"]["state"] = {
        "type" : "aerodynamic",
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 0.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    FM = scene.solve_forces()
    assert abs(FM["test_plane"]["total"]["FL"]-20.96187814976615)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.393693329077191)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.6612853313914597)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+20.997747935710734)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"])<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+12.88160169226603)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"])<1e-10

def test_nonlinear_NLL():
    # Tests the NLL algorithm is correctly solved

    # Alter input
    with open(input_file, 'r') as input_file_handle:
        input_dict = json.load(input_file_handle)

    input_dict["solver"]["type"] = "nonlinear"

    input_dict["scene"]["aircraft"]["test_plane"]["control_state"] = {
        "elevator" : 0.0,
        "rudder" : 0.0,
        "aileron" : 0.0
    }

    input_dict["scene"]["aircraft"]["test_plane"]["state"] = {
        "type" : "aerodynamic",
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 0.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    FM = scene.solve_forces()
    assert abs(FM["test_plane"]["total"]["FL"]-20.96342459325974)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.3936939776023654)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.6612320094219648)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+20.999293459785942)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"])<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+12.88084567119999)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"])<1e-10