# This script is for me to test the functionality of whatever I'm working on at the moment.
import machupX as MX
import json
import numpy as np
import subprocess as sp

if __name__=="__main__":

    input_file = "test/NLL_tests/input_for_NLL_testing.json"

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    altered_airplane_name = "airplane.json"
    input_dict["scene"]["aircraft"]["test_plane"]["file"] = altered_airplane_name

    with open(altered_airplane_name, 'w') as json_handle:
        json.dump(airplane_dict, json_handle)

    # Load scene
    scene = MX.Scene(input_dict)
    #scene.display_wireframe(show_legend=True)
    FM = scene.solve_forces(verbose=True)
    print(json.dumps(FM["test_plane"]["total"], indent=4))

    sp.run(["rm", altered_airplane_name])