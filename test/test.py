# This script is for me to test the functionality of whatever I'm working on at the moment.
import machupX as MX
import json
import numpy as np
import subprocess as sp
import matplotlib.pyplot as plt

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

    state["alpha"] = 2.0
    state["velocity"] = 200
    airplane_dict["wings"]["main_wing"]["chord"] = ["elliptic", 1.0]
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    scene.display_wireframe()

    print("Original state")
    FM = scene.solve_forces(non_dimensional=False)
    print(json.dumps(FM["test_plane"]["total"], indent=4))
    print(scene._airplanes["test_plane"].get_aerodynamic_state())

    trim_angles = scene.aircraft_pitch_trim(verbose=True, set_trim_state=True)
    print(json.dumps(trim_angles["test_plane"], indent=4))

    print("Trim state")
    FM = scene.solve_forces(non_dimensional=False)
    print(json.dumps(FM["test_plane"]["total"], indent=4))

    derivs = scene.aircraft_derivatives()
    print(json.dumps(derivs["test_plane"]["stability"], indent=4))