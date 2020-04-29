# Tests that the NLL algorithm is correctly solved

import machupX as MX
import numpy as np
import json
import subprocess as sp

input_file = "test/input_for_testing.json"


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
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 0.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    FM = scene.solve_forces()
    assert abs(FM["test_plane"]["total"]["FL"]-20.963419907083935)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.3936939776023654)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.6612320094219648)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+20.999293459785942)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"])<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+12.88084567119999)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"])<1e-10


def test_sideslip():
    # Tests the algorithm is correctly solved when there is sideslip

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
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 3.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    FM = scene.solve_forces()
    assert abs(FM["test_plane"]["total"]["FL"]-20.961937696103853)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.5650657667527752)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"]+4.067078748220425)<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.6178147806271342)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"]+4.143371186101563)<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+20.996289493261642)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"]+2.758992343363423)<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+13.02141786807216)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"]-12.325815049797061)<1e-10


def test_angular_rotation():
    # Tests the algorithm is correctly solved when there is angular rotation

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
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.1, 0.1],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 3.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    FM = scene.solve_forces()
    assert abs(FM["test_plane"]["total"]["FL"]-21.572483067700848)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.5716318622656185)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"]+3.8352520904652434)<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.6151782803754537)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"]+3.912205286677042)<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+21.607114950691514)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"]+2.4630018028476055)<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+14.743123116858015)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"]-11.627215607117007)<1e-10


def test_flap_deflection():
    # Tests the algorithm is correctly solved when there is flap deflection

    # Alter input
    with open(input_file, 'r') as input_file_handle:
        input_dict = json.load(input_file_handle)

    input_dict["solver"]["type"] = "linear"

    input_dict["scene"]["aircraft"]["test_plane"]["control_state"] = {
        "elevator" : -2.0,
        "rudder" : 0.0,
        "aileron" : 3.0
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
    FM = scene.solve_forces()
    assert abs(FM["test_plane"]["total"]["FL"]-15.380997116045302)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-2.1716305378887792)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"]-0.6544050217081607)<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+1.6335185811079909)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"]-0.6544050217081607)<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+15.447416240978942)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"]+9.609305988467975)<1e-10
    assert abs(FM["test_plane"]["total"]["My"]-3.6127056551475074)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"]+2.3148796481465355)<1e-10


def test_coefficients():
    # Tests the NLL algorithm correctly returns nondimensional coefficients

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
        "position" : [0, 0, 1000],
        "angular_rates" : [0.0, 0.0, 0.0],
        "velocity" : 100,
        "alpha" : 2.0,
        "beta" : 0.0
    }

    # Create scene
    scene = MX.Scene(input_dict)
    FM = scene.solve_forces(non_dimensional=True)
    assert abs(FM["test_plane"]["total"]["CL"]-0.22044634308263014)<1e-10
    assert abs(FM["test_plane"]["total"]["CD"]-0.014655821299887987)<1e-10
    assert abs(FM["test_plane"]["total"]["CS"])<1e-10
    assert abs(FM["test_plane"]["total"]["Cx"]+0.006953426946048488)<1e-10
    assert abs(FM["test_plane"]["total"]["Cy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Cz"]+0.22082353391381299)<1e-10
    assert abs(FM["test_plane"]["total"]["Cl"])<1e-10
    assert abs(FM["test_plane"]["total"]["Cm"]+0.13540656980424948)<1e-10
    assert abs(FM["test_plane"]["total"]["Cn"])<1e-10


def test_swept_wing():
    # Tests that the proper result for a swept wing is produced using the scipy solver

    # Inputs
    input_dict = {
        "solver" : {
            "type" : "scipy_fsolve"
        },
        "scene" : {}
    }
    airplane_dict = {
        "weight" : 10,
        "airfoils" : {
            "NACA_0012" : {
                "CLa" : 6.907213339669221,
                "geometry" : {
                    "NACA" : "0012"
                }
            }
        },
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "semispan" : 2.5,
                "chord" : 1.0,
                "airfoil" : "NACA_0012",
                "sweep" : 45.0,
                "ac_offset" : "kuchemann",
                "grid" : {
                    "N" : 20,
                    "reid_corrections" : True
                }
            }
        }
    }

    # Get results
    state = {
        "velocity" : 10.0,
        "alpha" : 10.0
    }
    scene = MX.Scene(input_dict)
    scene.add_aircraft("jackson_wing", airplane_dict, state=state)
    FM = scene.solve_forces(verbose=True, report_by_segment=True, non_dimensional=False)

    # Check circulation distribution
    correct_gamma = np.array([0.47123536, 1.36936314, 2.16971088, 2.78795243, 3.18301932, 3.41035458, 3.53474218, 3.59736899, 3.62115168, 3.61858063, 3.59673976, 3.56030944, 3.5136121,  3.46215726, 3.41358434, 3.37738807, 3.36088964, 3.35806733, 3.35540463, 3.353645,   3.353645,   3.35540463, 3.35806733, 3.36088964, 3.37738807, 3.41358434, 3.46215726, 3.5136121,  3.56030944, 3.59673976, 3.61858063, 3.62115168, 3.59736899, 3.53474218, 3.41035458, 3.18301932, 2.78795243, 2.16971088, 1.36936314, 0.47123536])
    assert np.allclose(scene._gamma, correct_gamma)
