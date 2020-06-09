# Tests that geometry is correctly initialized from the input file

import machupX as MX
import numpy as np
import json
import subprocess as sp


input_file = "test/input_for_testing.json"


def test_compute_straight_wing_root_location():
    # Tests the root of the wing is in the proper location for a wing with no 
    # sweep or dihedral

    # Load input
    input_dict, airplane_name, airplane_dict, state, control_state = MX.helpers.parse_input(input_file)

    # Alter input
    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key].pop("twist", None)

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft(airplane_name, airplane_dict, state=state, control_state=control_state)

    # Check wing root locations
    correct_locations = {
        "main_wing_left" : [0, 0, 0],
        "main_wing_right" : [0, 0, 0],
        "h_stab_left" : [-3.0, 0, 0],
        "h_stab_right" : [-3.0, 0, 0],
        "v_stab_right" : [-3.0, 0, -0.1]
    }

    wing_dict = scene._airplanes[airplane_name].wing_segments
    for key in wing_dict:
        root_loc = wing_dict[key].get_root_loc()
        assert np.allclose(root_loc.flatten(), correct_locations[key], rtol=0, atol=1e-10)


def test_compute_straight_wing_tip_location():
    # Tests the tip of the wing is in the proper location for a wing with no 
    # sweep or dihedral

    # Load input
    input_dict, airplane_name, airplane_dict, state, control_state = MX.helpers.parse_input(input_file)

    # Alter input
    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "v_stab" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key].pop("twist", None)

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft(airplane_name, airplane_dict, state=state, control_state=control_state)

    # Check wing tip locations
    correct_locations = {
        "main_wing_left" : [0, -4, 0],
        "main_wing_right" : [0, 4, 0],
        "h_stab_left" : [-3, -2, 0],
        "h_stab_right" : [-3, 2, 0],
        "v_stab_right" : [-3, 0, -2.1]
    }

    wing_dict = scene._airplanes[airplane_name].wing_segments
    for key in wing_dict:
        tip_loc = wing_dict[key].get_tip_loc()
        assert np.allclose(tip_loc.flatten(), correct_locations[key], rtol=0, atol=1e-10)


def test_compute_constant_swept_dihedral_wing_tip_location():
    # Tests the tip of the wing is in the proper location for a wing with 
    # both constant sweep and constant dihedral

    # Load input
    input_dict, airplane_name, airplane_dict, state, control_state = MX.helpers.parse_input(input_file)

    # Alter input
    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["sweep"] = 30
        if "v_stab" not in key:
            airplane_dict["wings"][key]["dihedral"] = 10
        airplane_dict["wings"][key].pop("twist", None)

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft(airplane_name, airplane_dict, state=state, control_state=control_state)

    # Check wing tip locations
    correct_locations = {
        "main_wing_left" : [-4.618802153517006/2, -7.878462024097664/2, -1.3891854213354426/2],
        "main_wing_right" : [-4.618802153517006/2, 7.878462024097664/2, -1.3891854213354426/2],
        "h_stab_left" : [-4.1547005383792515, -1.9696155060244158, -0.3472963553338607],
        "h_stab_right" : [-4.1547005383792515, 1.9696155060244158, -0.3472963553338607],
        "v_stab_right" : [-4.1547005383792515, 0, -2.1]
    }

    wing_dict = scene._airplanes[airplane_name].wing_segments
    for key in wing_dict:
        tip_loc = wing_dict[key].get_tip_loc()
        assert np.allclose(tip_loc.flatten(), correct_locations[key], rtol=0, atol=1e-10)


def test_compute_variable_swept_dihedral_wing_tip_location():
    # Tests the tip of the wing is in the proper location for a wing with 
    # both linearly varying sweep and linearly varying dihedral

    # Load input
    input_dict, airplane_name, airplane_dict, state, control_state = MX.helpers.parse_input(input_file)

    # Alter input
    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["sweep"] = "test/wing_segment_tests/sweep_dist.csv"
        if "v_stab" not in key:
            airplane_dict["wings"][key]["dihedral"] = [[0.0, 0.0],
                                                       [1.0, 10.0]]
        airplane_dict["wings"][key].pop("twist", None)

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft(airplane_name, airplane_dict, state=state, control_state=control_state)

    # Check wing tip locations
    correct_locations = {
        "main_wing_left" : [-1.5270189901562612, -3.979723080181195, -0.3481806534883265],
        "main_wing_right" : [-1.5270189901562612, 3.979723080181195, -0.3481806534883265],
        "h_stab_left" : [-3.7635094950781305, -1.9898615400905975, -0.17409032674416325],
        "h_stab_right" : [-3.7635094950781305, 1.9898615400905975, -0.17409032674416325],
        "v_stab_right" : [-3.7635094950781305, 0, -2.1]
    }

    wing_dict = scene._airplanes[airplane_name].wing_segments
    for key in wing_dict:
        tip_loc = wing_dict[key].get_tip_loc()
        assert np.allclose(tip_loc.flatten(), correct_locations[key], rtol=0, atol=1e-10)


def test_twist_as_f_of_span():
    # Tests that twist is properly returned as a function of span

    # Load input
    input_dict, airplane_name, airplane_dict, state, control_state = MX.helpers.parse_input(input_file)

    # Alter input
    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key]["twist"] = [[0.0, 0.0],
                                                    [0.5, 5.0],
                                                    [1.0, -5.0]]

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft(airplane_name, airplane_dict, state=state, control_state=control_state)

    spans = [0.0, 0.1, 0.2]
    twists = [0.0, 0.017453292519943295, 0.03490658503988659]

    wing_dict = scene._airplanes[airplane_name].wing_segments
    for key in wing_dict:
        for span, correct_twist in zip(spans, twists):
            twist = wing_dict[key].get_twist(span)
            print(twist)
            assert np.allclose(twist, correct_twist, rtol=0.0, atol=1e-10)


def test_node_generation():
    # Tests that the vortex nodes are properly generated

    # Load input
    input_dict, airplane_name, airplane_dict, state, control_state = MX.helpers.parse_input(input_file)

    # Alter input
    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key].pop("twist", None)

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft(airplane_name, airplane_dict, state=state, control_state=control_state)

    wing_dict = scene._airplanes[airplane_name].wing_segments
    for key in wing_dict:
        print(wing_dict[key].nodes)


def test_cp_generation():
    # Tests that the control points are properly generated

    # Load input
    input_dict, airplane_name, airplane_dict, state, control_state = MX.helpers.parse_input(input_file)

    # Alter input
    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key].pop("twist", None)

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft(airplane_name, airplane_dict, state=state, control_state=control_state)

    wing_dict = scene._airplanes[airplane_name].wing_segments
    for key in wing_dict:
        print(wing_dict[key].control_points)