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
        spans = np.random.random(5)
        for key in wing_segments:
            CL = wing_segments[key].get_CL(spans, np.repeat(alphas[np.newaxis], 5, axis=0))
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
        for i in range(5):
            span = np.random.random()
            for key in wing_segments:
                CD = wing_segments[key].get_CD(span, np.asarray(alpha))
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
        for i in range(5):
            span = np.random.random()
            for key in wing_segments:
                Cm = wing_segments[key].get_Cm(span, np.asarray(alpha))
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

    alphas = np.radians([0,1,2,3,4,5])
    spans = [0.0, 0.5, 1.0]
    CLs = np.asarray([[0.22807168, 0.11403584, 0.0],
                      [0.33624020572160057, 0.2207810497165992, 0.10532189371159782],
                      [0.44440873144320114, 0.3275262594331984, 0.210643787423195641],
                      [0.5525772571648018, 0.43427146914979764, 0.3159656811347935],
                      [0.6607457828864023, 0.5410166788663968, 0.4212875748463913],
                      [0.7689143086080029, 0.647761888582996, 0.5266094685579891]])

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for j, span in enumerate(spans):
            for key in wing_segments:
                CL = wing_segments[key].get_CL(span, np.asarray(alpha))
                assert np.allclose(CL, CLs[i,j], rtol=0.0, atol=1e-10)

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

    alphas = np.radians([0,1,2,3,4,5])
    spans = [0.0, 0.5, 1.0]
    CDs = np.asarray([[0.005204651028667432, 0.003602325514333716, 0.002],
                      [0.005352716824067322, 0.0037429146198036433, 0.0021331124155399652],
                      [0.00574415156256833, 0.004138300612364095, 0.002532449662159861],
                      [0.006378955244170457, 0.004788483492015072, 0.003198011739859688],
                      [0.0072571278688737025, 0.005693463258756574, 0.004129798648639445],
                      [0.008378669436678067, 0.0068532399125886, 0.005327810388499133]])

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for j, span in enumerate(spans):
            for key in wing_segments:
                CD = wing_segments[key].get_CD(span, alpha)
                assert np.allclose(CD, CDs[i,j], rtol=0.0, atol=1e-10)

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

    alphas = np.radians([0,1,2,3,4,5])
    spans = [0.0, 0.5, 1.0]
    Cms = np.asarray([[-0.0525, -0.02625, 0.0],
                      [-0.05193102266384984, -0.02595678468566495, 1.74532925e-05],
                      [-0.051362045327699696, -0.025663569371329905, 3.49065850e-05],
                      [-0.05079306799154954, -0.025370354056994854, 5.23598776e-05],
                      [-0.050224090655399393, -0.02507713874265981, 6.98131701e-05],
                      [-0.04965511331924924, -0.02478392342832476, 8.72664626e-05]])

    wing_segments = scene.airplanes["test_plane"].wing_segments
    for i, alpha in enumerate(alphas):
        for j, span in enumerate(spans):
            for key in wing_segments:
                Cm = wing_segments[key].get_Cm(span, alpha)
                assert np.allclose(Cm, Cms[i,j], rtol=0.0, atol=1e-10)
