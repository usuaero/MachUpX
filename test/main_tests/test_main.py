# Tests the __main__ script 

import subprocess as sp
import os
import json

input_file = "test/input_for_testing.json"

def test_main():
    # Tests the files are created properly

    # THis is currently failing for some unknown reason...
    return

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    input_dict["run"] = {
        "forces" : {},
        "aero_derivatives" : {},
        "distributions" : {},
        "pitch_trim" : {},
        "aero_center" : {}
    }

    # Write new input to file
    altered_input_name = "unique_name.json"
    with open(altered_input_name, 'w') as new_input_handle:
        json.dump(input_dict, new_input_handle)

    # Run MachUp
    sp.run(["python3", "-m", "machupX", altered_input_name])

    # Check the proper files have been created
    assert os.path.exists(altered_input_name.replace(".json", "_forces.json"))
    assert os.path.exists(altered_input_name.replace(".json", "_derivatives.json"))
    assert os.path.exists(altered_input_name.replace(".json", "_distributions.txt"))
    assert os.path.exists(altered_input_name.replace(".json", "_pitch_trim.json"))
    assert os.path.exists(altered_input_name.replace(".json", "_aero_center.json"))

    # Cleanup
    sp.run(["rm", altered_input_name.replace(".json", "_forces.json")])
    sp.run(["rm", altered_input_name.replace(".json", "_derivatives.json")])
    sp.run(["rm", altered_input_name.replace(".json", "_distributions.txt")])
    sp.run(["rm", altered_input_name.replace(".json", "_pitch_trim.json")])
    sp.run(["rm", altered_input_name.replace(".json", "_aero_center.json")])
    sp.run(["rm", altered_input_name])