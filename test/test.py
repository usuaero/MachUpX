# This script is for me to test the functionality of whatever I'm working on at the moment.
import machupX as MX
import json
import numpy as np
import subprocess as sp
import matplotlib.pyplot as plt
from stl import mesh
from mpl_toolkits import mplot3d

if __name__=="__main__":
    
    input_file = "test/input_for_testing.json"

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    airplane_state = input_dict["scene"]["aircraft"].pop("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})

    # Load scene
    scene = MX.Scene(input_dict)

    airplane_dict["wings"]["main_wing"]["airfoil"] = [[0.0, "NACA_4410"],
                                                      [0.5, "NACA_0010"],
                                                      [1.0, "NACA_0010"]]
    airplane_dict["wings"]["main_wing"]["dihedral"] = [[0.0, 0.0],
                                                       [0.5, 0.0],
                                                       [1.0, 20.0]]
    airplane_dict["wings"]["main_wing"]["sweep"] = [[0.0, 0.0],
                                                    [0.5, 0.0],
                                                    [0.5, 50.0],
                                                    [1.0, 50.0]]
   
    state["position"] = [0.0, 0.0, 0.0]
    state["orientation"] = [0.0, 0.0, 0.0]
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    # Create stl mesh
    scene.export_stl("wing.stl")