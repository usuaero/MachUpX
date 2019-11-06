# Tests the functionality of interpolating airfoil parameters along a wing segment

import machupX as MX
import numpy as np
import json

input_file = "test/input_for_testing.json"

def test_constant_airfoil_section_get_lift():
    # Tests the lift coefficient is properly given for a constant airfoil section

    # Load scene
    scene = MX.Scene(input_file)

    alphas = np.radians([0,1,2])
    CLs = [0.0, 0.1122875027563072, 0.2245750055126144]

    wing_segments = scene._airplanes["test_plane"].wing_segments
    for alpha, correct_CL in zip(alphas, CLs):
        for key in wing_segments:
            N = wing_segments[key]._N
            params = np.concatenate((np.repeat(alpha, N)[:,np.newaxis], np.zeros((N,1)), np.zeros((N,1))), axis=1)
            CL = wing_segments[key].get_cp_CL(params)
            assert np.allclose(CL, correct_CL, rtol=0.0, atol=1e-10)

def test_constant_airfoil_section_get_drag():
    # Tests the drag coefficient is properly given for a constant airfoil section

    # Load scene
    scene = MX.Scene(input_file)

    alphas = np.radians([0,1,2])
    CDs = [0.00513, 0.0063706747542843735, 0.010092699017137493]

    wing_segments = scene._airplanes["test_plane"].wing_segments
    for alpha, correct_CD in zip(alphas, CDs):
        for key in wing_segments:
            N = wing_segments[key]._N
            params = np.concatenate((np.repeat(alpha, N)[:,np.newaxis], np.zeros((N,1)), np.zeros((N,1))), axis=1)
            CD = wing_segments[key].get_cp_CD(params)
            assert np.allclose(CD, correct_CD, rtol=0.0, atol=1e-10)

def test_constant_airfoil_section_get_moment():
    # Tests the moment coefficient is properly given for a constant airfoil section

    # Load scene
    scene = MX.Scene(input_file)

    alphas = np.radians([0,1,2])
    Cms = [0.0, 0.0, 0.0]

    wing_segments = scene._airplanes["test_plane"].wing_segments
    for alpha, correct_Cm in zip(alphas, Cms):
        for key in wing_segments:
            N = wing_segments[key]._N
            params = np.concatenate((np.repeat(alpha, N)[:,np.newaxis], np.zeros((N,1)), np.zeros((N,1))), axis=1)
            Cm = wing_segments[key].get_cp_Cm(params)
            assert np.allclose(Cm, correct_Cm, rtol=0.0, atol=1e-10)

def test_variable_airfoil_section_get_lift():
    # Tests the lift coefficient is properly given for a variable airfoil section

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["airfoil"] = [[0.0, "NACA_2410"],
                                                  [1.0, "NACA_0010"]]
        airplane_dict["wings"][key]["grid"]["N"] = 5

    airplane_state = input_dict["scene"]["aircraft"].get("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    alphas = np.radians([0,1,2])
    CLs = np.asarray([[0.00558131, 0.04700726, 0.11403584, 0.18106442, 0.22249037],
                      [0.11776802, 0.15844581, 0.22426385, 0.2900819 , 0.33075969],
                      [0.22995472, 0.26988436, 0.33449187, 0.39909938, 0.43902902]])

    wing_segments = scene._airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for key in wing_segments:
            N = wing_segments[key]._N
            params = np.concatenate((np.repeat(alpha, N)[:,np.newaxis], np.zeros((N,1)), np.zeros((N,1))), axis=1)
            CL = wing_segments[key].get_cp_CL(params)
            if wing_segments[key]._side == "left":
                assert np.allclose(CL, CLs[i], rtol=0.0, atol=1e-8)
            else:
                assert np.allclose(CL, CLs[i,::-1], rtol=0.0, atol=1e-8)

def test_variable_airfoil_section_get_drag():
    # Tests the drag coefficient is properly given for a variable airfoil section

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["airfoil"] = [[0.0, "NACA_2410"],
                                                  [1.0, "NACA_0010"]]
        airplane_dict["wings"][key]["grid"]["N"] = 5

    airplane_state = input_dict["scene"]["aircraft"].get("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    alphas = np.radians([0,1,2])
    spans = np.asarray([0.0, 0.5, 1.0])
    CDs = np.asarray([[0.00513183, 0.00514539, 0.00516733, 0.00518926, 0.00520282],
                      [0.00634576, 0.00616087, 0.0058617 , 0.00556253, 0.00537763],
                      [0.00998628, 0.00919643, 0.00791843, 0.00664042, 0.00585057]])

    wing_segments = scene._airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for key in wing_segments:
            N = wing_segments[key]._N
            params = np.concatenate((np.repeat(alpha, N)[:,np.newaxis], np.zeros((N,1)), np.zeros((N,1))), axis=1)
            CD = wing_segments[key].get_cp_CD(params)
            if wing_segments[key]._side == "left":
                assert np.allclose(CD, CDs[i], rtol=0.0, atol=1e-8)
            else:
                assert np.allclose(CD, CDs[i,::-1], rtol=0.0, atol=1e-8)

def test_variable_airfoil_section_get_moment():
    # Tests the moment coefficient is properly given for a variable airfoil section

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["airfoil"] = [[0.0, "NACA_2410"],
                                                  [1.0, "NACA_0010"]]
        airplane_dict["wings"][key]["grid"]["N"] = 5

    airplane_state = input_dict["scene"]["aircraft"].get("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    alphas = np.radians([0,1,2])
    Cms = np.asarray([[-0.00128477, -0.01082064, -0.02625   , -0.04167936, -0.05121523],
                      [-0.00127084, -0.01070337, -0.02596551, -0.04122766, -0.05066018],
                      [-0.00125692, -0.0105861 , -0.02568102, -0.04077595, -0.05010513]])

    wing_segments = scene._airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for key in wing_segments:
            N = wing_segments[key]._N
            params = np.concatenate((np.repeat(alpha, N)[:,np.newaxis], np.zeros((N,1)), np.zeros((N,1))), axis=1)
            Cm = wing_segments[key].get_cp_Cm(params)
            if wing_segments[key]._side == "left":
                assert np.allclose(Cm, Cms[i], rtol=0.0, atol=1e-8)
            else:
                assert np.allclose(Cm, Cms[i,::-1], rtol=0.0, atol=1e-8)

def test_variable_airfoil_section_from_file_get_lift():
    # Tests the lift coefficient is properly given for a variable airfoil section defined by a file

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["airfoil"] = "test/wing_segment_tests/airfoil_dist.csv"
        airplane_dict["wings"][key]["grid"]["N"] = 5

    airplane_state = input_dict["scene"]["aircraft"].get("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    alphas = np.radians([0,1,2])
    CLs = np.asarray([[0.00558131, 0.04700726, 0.11403584, 0.18106442, 0.22249037],
                      [0.11776802, 0.15844581, 0.22426385, 0.2900819 , 0.33075969],
                      [0.22995472, 0.26988436, 0.33449187, 0.39909938, 0.43902902]])

    wing_segments = scene._airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for key in wing_segments:
            N = wing_segments[key]._N
            params = np.concatenate((np.repeat(alpha, N)[:,np.newaxis], np.zeros((N,1)), np.zeros((N,1))), axis=1)
            CL = wing_segments[key].get_cp_CL(params)
            if wing_segments[key]._side == "left":
                assert np.allclose(CL, CLs[i], rtol=0.0, atol=1e-8)
            else:
                assert np.allclose(CL, CLs[i,::-1], rtol=0.0, atol=1e-8)