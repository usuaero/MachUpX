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

<<<<<<< HEAD
    input_dict["solver"]["type"] = "linear"

=======
>>>>>>> v1.0.1-dev
    airplane_state = input_dict["scene"]["aircraft"].pop("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})

    airplane_dict["wings"]["main_wing"]["sweep"] = [[0.0, 0.0],
                                                    [0.2, 15.0],
                                                    [1.0, 40.0]]
    airplane_dict["wings"]["main_wing"]["dihedral"] = [[0.0, 0.0],
                                                       [0.8, 10.0],
                                                       [0.9, 65.0],
                                                       [1.0, 70.0]]
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
    airplane_dict["wings"]["main_wing"]["grid"]["N"] = 50
    airplane_dict["wings"].pop("v_stab")
    airplane_dict["wings"].pop("h_stab")

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)

    state["alpha"] = 2.0
    state["beta"] = 0.0
    state["velocity"] = 200
    airplane_dict["wings"]["main_wing"]["chord"] = [[0.0, 1.0],[0.9, 1.0],[1.0, 0.5]]
    #airplane_dict["wings"]["main_wing"]["chord"] = ["elliptic", 1.0]
    airplane_dict["wings"]["main_wing"]["semispan"] = 4.0
    airplane_dict["wings"].pop("v_stab")
    airplane_dict["wings"].pop("h_stab")
    #airplane_dict["wings"]["v_stab"]["chord"] = ["elliptic", 1.0]
    #airplane_dict["wings"]["v_stab"]["sweep"] = 0.0
    #airplane_dict["wings"]["h_stab"]["chord"] = ["elliptic", 1.0]
    #airplane_dict["wings"]["h_stab"]["sweep"] = 0.0
    airplane_dict["wings"]["main_wing"]["dihedral"] = 0.
    airplane_dict["wings"]["main_wing"]["sweep"] = 45.
    airplane_dict["wings"]["main_wing"]["grid"]["N"] = 100
    airplane_dict["wings"]["main_wing"]["grid"]["flap_edge_cluster"] = True
    airplane_dict["wings"]["main_wing"]["control_surface"]["root_span"] = 0.4
    airplane_dict["wings"]["main_wing"]["control_surface"]["tip_span"] = 0.9

    control_state["aileron"] = 20
   
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    scene.display_wireframe()

    print("Original state")
    FM = scene.solve_forces(non_dimensional=False, verbose=True)
    print(json.dumps(FM["test_plane"]["total"], indent=4))

    #trim_angles = scene.aircraft_pitch_trim(verbose=True, set_trim_state=True)
    #print(json.dumps(trim_angles["test_plane"], indent=4))

    #print("---Trim State---")
    #FM = scene.solve_forces(non_dimensional=False, verbose=True)
    #print(json.dumps(FM["test_plane"]["total"], indent=4))

    #print("---Derivatives---")
    #derivs = scene.aircraft_derivatives()
    #print(json.dumps(derivs["test_plane"]["stability"], indent=4))

    print("---MAC---")
    MAC = scene.aircraft_mean_aerodynamic_chord()
    print(json.dumps(MAC["test_plane"], indent=4))

    print("---Aerodynamic Center---")
    AC = scene.aircraft_aero_center()
    print(json.dumps(AC["test_plane"], indent=4))
    
    scene.export_aircraft_stp("plane", section_resolution=50, spline=True, maintain_sections=True)