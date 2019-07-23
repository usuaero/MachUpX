# Tests that geometry is correctly initialized from the input file

import machupX as MX
import numpy as np
import json
import subprocess as sp

input_file = "test/airplane_tests/input_for_airplane_testing.json"

def test_all_wing_segments_get_added():
    # Tests that all wing segments have been added and are available through 
    # both the tree and the dict.

    # Load wing segments
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    wing_segments = []
    wing_segment_sides = []
    for key in airplane_dict["wings"]:
        wing_segments.append(key)
        wing_segment_sides.append(airplane_dict["wings"][key]["side"])

    # Create scene
    scene = MX.Scene(input_file)

    # Check for each wing segment in both the tree and the dict
    wing_dict = scene.airplanes["test_plane"]._wing_segments
    for segment_name, segment_side in zip(wing_segments, wing_segment_sides):

        if segment_side == "left" or segment_side == "both":
            name = segment_name+"_left"

            # Retrieve from dictionary
            dict_segment = wing_dict[name]

            # Retrieve from tree
            tree_segment = scene.airplanes["test_plane"]._get_wing_segment(name)

            # Make sure these reference the same object
            assert dict_segment is tree_segment

        if segment_side == "right" or segment_side == "both":
            name = segment_name+"_right"

            # Retrieve from dictionary
            dict_segment = wing_dict[name]

            # Retrieve from tree
            tree_segment = scene.airplanes["test_plane"]._get_wing_segment(name)

            # Make sure these reference the same object
            assert dict_segment is tree_segment

def test_compute_straight_wing_root_location():
    # Tests the root of the wing is in the proper location for a wing with no 
    # sweep or dihedral

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key].pop("twist", None)

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    # Check wing root locations
    correct_locations = {
        "main_wing_inside_left" : [0, 0, 0],
        "main_wing_inside_right" : [0, 0, 0],
        "main_wing_outside_left" : [0, -8, 0],
        "main_wing_outside_right" : [0, 8, 0],
        "horizontal_stabilizer_left" : [-10, 0, 0],
        "horizontal_stabilizer_right" : [-10, 0, 0],
        "vertical_stabilizer_right" : [-10, 0, 0]
    }

    wing_dict = scene.airplanes["test_plane"]._wing_segments
    for key in wing_dict:
        root_loc = wing_dict[key].get_root_loc()
        assert np.allclose(root_loc.flatten(), correct_locations[key], rtol=0, atol=1e-10)

    sp.run(["rm", altered_airplane_name])
    sp.run(["rm", altered_input_name])

def test_compute_straight_wing_tip_location():
    # Tests the tip of the wing is in the proper location for a wing with no 
    # sweep or dihedral

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key].pop("twist", None)

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    # Check wing tip locations
    correct_locations = {
        "main_wing_inside_left" : [0, -8, 0],
        "main_wing_inside_right" : [0, 8, 0],
        "main_wing_outside_left" : [0, -16, 0],
        "main_wing_outside_right" : [0, 16, 0],
        "horizontal_stabilizer_left" : [-10, -6, 0],
        "horizontal_stabilizer_right" : [-10, 6, 0],
        "vertical_stabilizer_right" : [-10, 0, -6]
    }

    wing_dict = scene.airplanes["test_plane"]._wing_segments
    for key in wing_dict:
        tip_loc = wing_dict[key].get_tip_loc()
        assert np.allclose(tip_loc.flatten(), correct_locations[key], rtol=0, atol=1e-10)

    sp.run(["rm", altered_airplane_name])
    sp.run(["rm", altered_input_name])

def test_compute_constant_swept_dihedral_wing_tip_location():
    # Tests the tip of the wing is in the proper location for a wing with 
    # both constant sweep and constant dihedral

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["sweep"] = 30
        if "vertical" not in key:
            airplane_dict["wings"][key]["dihedral"] = 10
        airplane_dict["wings"][key].pop("twist", None)

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    # Check wing tip locations
    correct_locations = {
        "main_wing_inside_left" : [-4.618802153517006, -7.878462024097664, -1.3891854213354426],
        "main_wing_inside_right" : [-4.618802153517006, 7.878462024097664, -1.3891854213354426],
        "main_wing_outside_left" : [-9.237604307034012, -15.756924048195328, -2.7783708426708853],
        "main_wing_outside_right" : [-9.237604307034012, 15.756924048195328, -2.7783708426708853],
        "horizontal_stabilizer_left" : [-13.4641016151377544, -5.908846518073248, -1.041889066001582],
        "horizontal_stabilizer_right" : [-13.4641016151377544, 5.908846518073248, -1.041889066001582],
        "vertical_stabilizer_right" : [-13.4641016151377544, 0, -6]
    }

    wing_dict = scene.airplanes["test_plane"]._wing_segments
    for key in wing_dict:
        tip_loc = wing_dict[key].get_tip_loc()
        assert np.allclose(tip_loc.flatten(), correct_locations[key], rtol=0, atol=1e-10)

    sp.run(["rm", altered_airplane_name])
    sp.run(["rm", altered_input_name])

def test_compute_variable_swept_dihedral_wing_tip_location():
    # Tests the tip of the wing is in the proper location for a wing with 
    # both linearly varying sweep and linearly varying dihedral

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key]["sweep"] = [[0.0, 0.0],
                                                [1.0, 40.0]]
        if "vertical" not in key:
            airplane_dict["wings"][key]["dihedral"] = [[0.0, 0.0],
                                                       [1.0, 10.0]]
        airplane_dict["wings"][key].pop("twist", None)

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    # Check wing tip locations
    correct_locations = {
        "main_wing_inside_left" : [-3.054037980312523, -7.95944616036239, -0.696361306976653],
        "main_wing_inside_right" : [-3.054037980312523, 7.95944616036239, -0.696361306976653],
        "main_wing_outside_left" : [-6.108075960625046, -15.91889232072478, -1.392722613953306],
        "main_wing_outside_right" : [-6.108075960625046, 15.91889232072478, -1.392722613953306],
        "horizontal_stabilizer_left" : [-12.2905284852343923, -5.969584620271792, -0.5222709802324897],
        "horizontal_stabilizer_right" : [-12.2905284852343923, 5.969584620271792, -0.5222709802324897],
        "vertical_stabilizer_right" : [-12.2905284852343923, 0, -6]
    }

    wing_dict = scene.airplanes["test_plane"]._wing_segments
    for key in wing_dict:
        tip_loc = wing_dict[key].get_tip_loc()
        assert np.allclose(tip_loc.flatten(), correct_locations[key], rtol=0, atol=1e-10)

    sp.run(["rm", altered_airplane_name])
    sp.run(["rm", altered_input_name])

def test_twist_as_f_of_span():
    # Tests that twist is properly returned as a function of span

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key]["twist"] = [[0.0, 0.0],
                                                    [0.5, 5.0],
                                                    [1.0, -5.0]]

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    spans = [0.0, 0.1, 0.2]
    twists = [0.0, 0.017453292519943295, 0.03490658503988659]

    wing_dict = scene.airplanes["test_plane"]._wing_segments
    for key in wing_dict:
        for span, correct_twist in zip(spans, twists):
            twist = wing_dict[key].get_twist(span).item()
            print(twist)
            assert np.allclose(twist, correct_twist, rtol=0.0, atol=1e-10)

    sp.run(["rm", altered_airplane_name])
    sp.run(["rm", altered_input_name])

def test_node_generation():
    # Tests that the vortex nodes are properly generated

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key].pop("twist", None)

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    wing_dict = scene.airplanes["test_plane"]._wing_segments
    for key in wing_dict:
        print(wing_dict[key].get_node_locs())

    sp.run(["rm", altered_airplane_name])
    sp.run(["rm", altered_input_name])

def test_cp_generation():
    # Tests that the control points are properly generated

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    for key in airplane_dict["wings"]:
        airplane_dict["wings"][key].pop("sweep", None)
        if "vertical" not in key:
            airplane_dict["wings"][key].pop("dihedral", None)
        airplane_dict["wings"][key].pop("twist", None)

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name
    altered_input_name = "input.json"

    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict, json_handle)

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(altered_input_name)

    wing_dict = scene.airplanes["test_plane"]._wing_segments
    for key in wing_dict:
        print(wing_dict[key].get_cp_locs())

    sp.run(["rm", altered_airplane_name])
    sp.run(["rm", altered_input_name])