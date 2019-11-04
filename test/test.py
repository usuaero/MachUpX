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

    airplane_dict["wings"]["main_wing"]["sweep"] = [[0.0, -10.0],[0.3, 0.0],[1.0, 30.0]]
    airplane_dict["wings"]["main_wing"]["dihedral"] = [[0.0, -20.0],[0.2, 0.0],[0.9, 0.0],[1.0, 80.0]]
    airplane_dict["wings"]["main_wing"]["chord"] = [[0,    0.775],
                                                    [0.01, 0.773800639863482],
                                                    [0.02, 0.77021023126586],
                                                    [0.03, 0.764251740569511],
                                                    [0.04, 0.755963281780915],
                                                    [0.05, 0.745397872751082],
                                                    [0.06, 0.732623096042232],
                                                    [0.07, 0.717720666630031],
                                                    [0.08, 0.70078590920661],
                                                    [0.09, 0.681927148427836],
                                                    [0.1,  0.661265016005187],
                                                    [0.11, 0.638931679074468],
                                                    [0.12, 0.615069994777171],
                                                    [0.13, 0.589832596462292],
                                                    [0.14, 0.563380917353767],
                                                    [0.15, 0.535884157928753],
                                                    [0.16, 0.507518203611983],
                                                    [0.17, 0.478464499709282],
                                                    [0.18, 0.448908890776805],
                                                    [0.19, 0.419040431850092],
                                                    [0.2,  0.389050179137017],
                                                    [0.9,  0.188],
                                                    [1,    0.06275]]
    airplane_dict["wings"]["main_wing"]["grid"]["N"] = 40
    #airplane_dict["wings"]["main_wing"]["chord"] = [[0.0, 1.0],[1.0, 1.0]]
    airplane_dict["wings"].pop("v_stab")
    airplane_dict["wings"].pop("h_stab")

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)

    scene.export_aircraft_stp("plane", section_resolution=50)