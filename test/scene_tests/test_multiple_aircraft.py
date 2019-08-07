# Tests the functionality of having multiple aircraft in a scene

import json
import machupX as MX
import numpy as np

input_file = "test/input_for_testing.json"

def test_add_two_aircraft():

    # Load input
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

    FM = scene.solve_forces(verbose=True)

    # Since the aircraft are mirror images, their forces should be the same, with some being opposite
    assert abs(FM["test_plane_0"]["total"]["FL"] - FM["test_plane_1"]["total"]["FL"])<1e-6
    assert abs(FM["test_plane_0"]["total"]["FD"] - FM["test_plane_1"]["total"]["FD"])<1e-6
    assert abs(FM["test_plane_0"]["total"]["FS"] + FM["test_plane_1"]["total"]["FS"])<1e-6
    assert abs(FM["test_plane_0"]["total"]["Fx"] - FM["test_plane_1"]["total"]["Fx"])<1e-6
    assert abs(FM["test_plane_0"]["total"]["Fy"] + FM["test_plane_1"]["total"]["Fy"])<1e-6
    assert abs(FM["test_plane_0"]["total"]["Fz"] - FM["test_plane_1"]["total"]["Fz"])<1e-6
    assert abs(FM["test_plane_0"]["total"]["Mx"] + FM["test_plane_1"]["total"]["Mx"])<1e-6
    assert abs(FM["test_plane_0"]["total"]["My"] - FM["test_plane_1"]["total"]["My"])<1e-6
    assert abs(FM["test_plane_0"]["total"]["Mz"] + FM["test_plane_1"]["total"]["Mz"])<1e-6