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

    input_dict["solver"]["type"] = "nonlinear"

    airplane_state = input_dict["scene"]["aircraft"].pop("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})

    # Load scene
    scene = MX.Scene(input_dict)

    state["orientation"] = [np.sqrt(2)/2, 0, 0, np.sqrt(2)/2]
    scene.add_aircraft("test_plane_0", airplane_dict, state=state, control_state=control_state)

    state["position"] = [50, 0, 0]
    scene.add_aircraft("test_plane_1", airplane_dict, state=state, control_state=control_state)

    scene.display_wireframe()

    FM = scene.solve_forces(verbose=True)

    print("Tail")
    print(json.dumps(FM["test_plane_0"]["total"], indent=4))
    print("Lead")
    print(json.dumps(FM["test_plane_1"]["total"], indent=4))