# This script is for me to test the functionality of whatever I'm working on at the moment.
import machupX as MX
import json
import numpy as np
import subprocess as sp

if __name__=="__main__":

    input_file = "test/input_for_testing.json"

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
    spans = np.arange(11)/10
    for key in wing_dict:
        print(wing_dict[key].get_section_ac_loc(spans))

    sp.run(["rm", altered_airplane_name])
    sp.run(["rm", altered_input_name])