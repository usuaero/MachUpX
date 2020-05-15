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
    assert abs(FM["test_plane"]["total"]["FL"]-20.95753393799707)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.393410892863983)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.661154678035072)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+20.993396513436387)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"])<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+12.874635754621892)<1e-10
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
    assert abs(FM["test_plane"]["total"]["FL"]-20.959074770349314)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.3934114313254868)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.661101441894963)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+20.994936425947238)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"])<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+12.873864820066961)<1e-10
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
    print(json.dumps(FM["test_plane"]["total"], indent=4))
    assert abs(FM["test_plane"]["total"]["FL"]-20.94156333437601)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.5649021699745385)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"]+4.054215389085199)<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.6189035316196642)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"]+4.13055987928926)<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+20.97594073248021)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"]+3.214692050656253)<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+13.059105060454414)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"]-12.279858552678329)<1e-10


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
    print(json.dumps(FM["test_plane"]["total"], indent=4))
    assert abs(FM["test_plane"]["total"]["FL"]-21.54799507617886)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.5713237939345306)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"]+3.8228634171595606)<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.6162489366718589)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"]+3.8998610490311862)<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+21.582649420795637)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"]+2.924780737323989)<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+14.7643635669839723)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"]-11.58247691212459)<1e-10


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
    assert abs(FM["test_plane"]["total"]["FL"]-15.371272228307886)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-2.1142210894279376)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"]-0.642970343115379)<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+1.5764834986195249)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"]-0.642970343115379)<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+15.435693716523126)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"]+9.66438577881925)<1e-10
    assert abs(FM["test_plane"]["total"]["My"]-2.809683822032028)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"]+0.24070673598985626)<1e-10


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
    assert abs(FM["test_plane"]["total"]["CL"]-0.22043013669655828)<1e-10
    assert abs(FM["test_plane"]["total"]["CD"]-0.014655815636380912)<1e-10
    assert abs(FM["test_plane"]["total"]["CS"])<1e-10
    assert abs(FM["test_plane"]["total"]["Cx"]+0.006953986880708745)<1e-10
    assert abs(FM["test_plane"]["total"]["Cy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Cz"]+0.22080733720257997)<1e-10
    assert abs(FM["test_plane"]["total"]["Cl"])<1e-10
    assert abs(FM["test_plane"]["total"]["Cm"]+0.13541467844956412)<1e-10
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
