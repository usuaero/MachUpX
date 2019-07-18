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

    wing_segments = scene.airplanes["test_plane"]._wing_segments
    for alpha, correct_CL in zip(alphas, CLs):
        for i in range(5):
            span = np.random.random()
            for key in wing_segments:
                CL = wing_segments[key].get_CL(span, alpha)
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

    wing_segments = scene.airplanes["test_plane"]._wing_segments
    for alpha, correct_CD in zip(alphas, CDs):
        for i in range(5):
            span = np.random.random()
            for key in wing_segments:
                CD = wing_segments[key].get_CD(span, alpha)
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

    wing_segments = scene.airplanes["test_plane"]._wing_segments
    for alpha, correct_Cm in zip(alphas, Cms):
        for i in range(5):
            span = np.random.random()
            for key in wing_segments:
                Cm = wing_segments[key].get_Cm(span, alpha)
                assert np.allclose(Cm, correct_Cm, rtol=0.0, atol=1e-10)