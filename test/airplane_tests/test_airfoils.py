# Tests the functionality of airfoil getters

import machupX as MX
import numpy as np
import json

input_file = "test/airplane_tests/input_for_airplane_testing.json"

def test_constant_airfoil_section_get_lift():
    # Tests the lift coefficient is properly given for a constant airfoil section

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["airfoil"] = "NACA_0010"

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    alphas = np.radians([0,1,2,3,4,5])
    CLs = [0.0, 0.10532189371159782, 0.21064378742319564, 0.3159656811347935, 0.4212875748463913, 0.5266094685579891]

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for alpha, correct_CL in zip(alphas, CLs):
        for key in wing_segments:
            CL = wing_segments[key].get_cp_CL(np.repeat(alpha[np.newaxis], wing_segments[key]._N))
            assert np.allclose(CL, correct_CL, rtol=0.0, atol=1e-10)

def test_constant_airfoil_section_get_drag():
    # Tests the drag coefficient is properly given for a constant airfoil section

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["airfoil"] = "NACA_0010"

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    alphas = np.radians([0,1,2,3,4,5])
    CDs = [0.002, 0.0021331124155399652, 0.002532449662159861, 0.003198011739859688, 0.004129798648639445, 0.005327810388499133]

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for alpha, correct_CD in zip(alphas, CDs):
        for key in wing_segments:
            CD = wing_segments[key].get_cp_CD(np.repeat(alpha[np.newaxis], wing_segments[key]._N))
            assert np.allclose(CD, correct_CD, rtol=0.0, atol=1e-10)

def test_constant_airfoil_section_get_moment():
    # Tests the moment coefficient is properly given for a constant airfoil section

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["airfoil"] = "NACA_0010"

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    alphas = np.radians([0,1,2,3,4,5])
    Cms = [0.0, 1.74532925e-05, 3.49065850e-05, 5.23598776e-05, 6.98131701e-05, 8.72664626e-05]

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for alpha, correct_Cm in zip(alphas, Cms):
        for key in wing_segments:
            Cm = wing_segments[key].get_cp_Cm(np.repeat(alpha[np.newaxis], wing_segments[key]._N))
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

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    alphas = np.radians([0,1,2])
    CLs = np.asarray([[0.00558131, 0.04700726, 0.11403584, 0.18106442, 0.22249037],
                      [0.11097287, 0.15291586, 0.22078105, 0.28864624, 0.33058923],
                      [0.21636442, 0.25882447, 0.32752626, 0.39622805, 0.4386881 ]])

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for key in wing_segments:
            CL = wing_segments[key].get_cp_CL(np.repeat(alpha[np.newaxis], wing_segments[key]._N))
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

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    alphas = np.radians([0,1,2])
    spans = np.asarray([0.0, 0.5, 1.0])
    CDs = np.asarray([[0.00207842, 0.0026605 , 0.00360233, 0.00454415, 0.00512623],
                      [0.0022119 , 0.0027967 , 0.00374291, 0.00468913, 0.00527393],
                      [0.00261105, 0.00319441, 0.0041383 , 0.0050822 , 0.00566556]])

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for key in wing_segments:
            CD = wing_segments[key].get_cp_CD(np.repeat(alpha[np.newaxis],wing_segments[key]._N))
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

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    alphas = np.radians([0,1,2])
    Cms = np.asarray([[-0.00128477, -0.01082064, -0.02625   , -0.04167936, -0.05121523],
                      [-0.00125382, -0.01068951, -0.02595678, -0.04122406, -0.05065975],
                      [-0.00122287, -0.01055838, -0.02566357, -0.04076875, -0.05010427]])

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for key in wing_segments:
            Cm = wing_segments[key].get_cp_Cm(np.repeat(alpha[np.newaxis],wing_segments[key]._N))
            if wing_segments[key]._side == "left":
                assert np.allclose(Cm, Cms[i], rtol=0.0, atol=1e-8)
            else:
                assert np.allclose(Cm, Cms[i,::-1], rtol=0.0, atol=1e-8)