# Tests the functionality of having multiple aircraft in a scene

import json
import machupX as MX
import numpy as np

input_file = "test/input_for_testing.json"


def test_two_aircraft():
    # Tests that two aircraft flying side by side are properly modelled

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

    state["position"] = [25, 0, 0]
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


def test_add_two_aircraft_really_far_away():
    # Test that two aircraft really far away from each other are almost the same as a single aircraft

    # Load input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    input_dict["solver"]["type"] = "nonlinear"

    airplane_state = input_dict["scene"]["aircraft"].pop("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})


    # Load single scene
    scene = MX.Scene(input_dict)

    state["orientation"] = [np.sqrt(2)/2, 0, 0, np.sqrt(2)/2]
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    single_FM = scene.solve_forces(verbose=True)

    # Load double scene
    scene = MX.Scene(input_dict)

    state["orientation"] = [np.sqrt(2)/2, 0, 0, np.sqrt(2)/2]
    scene.add_aircraft("test_plane_0", airplane_dict, state=state, control_state=control_state)

    state["position"] = [10000, 0, 0]
    scene.add_aircraft("test_plane_1", airplane_dict, state=state, control_state=control_state)

    double_FM = scene.solve_forces(verbose=True)

    # Since the planes are really far away, they should behave as if they're by themselves
    assert abs(double_FM["test_plane_0"]["total"]["FL"] - single_FM["test_plane"]["total"]["FL"])<1e-6
    assert abs(double_FM["test_plane_0"]["total"]["FD"] - single_FM["test_plane"]["total"]["FD"])<1e-6
    assert abs(double_FM["test_plane_0"]["total"]["FS"] - single_FM["test_plane"]["total"]["FS"])<1e-6
    assert abs(double_FM["test_plane_0"]["total"]["Fx"] - single_FM["test_plane"]["total"]["Fx"])<1e-6
    assert abs(double_FM["test_plane_0"]["total"]["Fy"] - single_FM["test_plane"]["total"]["Fy"])<1e-6
    assert abs(double_FM["test_plane_0"]["total"]["Fz"] - single_FM["test_plane"]["total"]["Fz"])<1e-6
    assert abs(double_FM["test_plane_0"]["total"]["Mx"] - single_FM["test_plane"]["total"]["Mx"])<1e-6
    assert abs(double_FM["test_plane_0"]["total"]["My"] - single_FM["test_plane"]["total"]["My"])<1e-6
    assert abs(double_FM["test_plane_0"]["total"]["Mz"] - single_FM["test_plane"]["total"]["Mz"])<1e-6

    assert abs(double_FM["test_plane_1"]["total"]["FL"] - single_FM["test_plane"]["total"]["FL"])<1e-6
    assert abs(double_FM["test_plane_1"]["total"]["FD"] - single_FM["test_plane"]["total"]["FD"])<1e-6
    assert abs(double_FM["test_plane_1"]["total"]["FS"] - single_FM["test_plane"]["total"]["FS"])<1e-6
    assert abs(double_FM["test_plane_1"]["total"]["Fx"] - single_FM["test_plane"]["total"]["Fx"])<1e-6
    assert abs(double_FM["test_plane_1"]["total"]["Fy"] - single_FM["test_plane"]["total"]["Fy"])<1e-6
    assert abs(double_FM["test_plane_1"]["total"]["Fz"] - single_FM["test_plane"]["total"]["Fz"])<1e-6
    assert abs(double_FM["test_plane_1"]["total"]["Mx"] - single_FM["test_plane"]["total"]["Mx"])<1e-6
    assert abs(double_FM["test_plane_1"]["total"]["My"] - single_FM["test_plane"]["total"]["My"])<1e-6
    assert abs(double_FM["test_plane_1"]["total"]["Mz"] - single_FM["test_plane"]["total"]["Mz"])<1e-6