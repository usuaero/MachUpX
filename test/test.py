# This script is for me to test the functionality of whatever I'm working on at the moment.
import machupX as MX
import json
import numpy as np
import subprocess as sp
import matplotlib.pyplot as plt
from stl import mesh
from mpl_toolkits import mplot3d

if __name__=="__main__":
    
    # Specify input
    input_dict = {
        "solver" : {
            "type" : "linear",
        },
        "scene" : {
        }
    }

    joint_length = 0.15

    # Specify airplane
    airplane_dict = {
        "CG" : [0,0,0],
        "weight" : 10.0,
        "controls" : {
            "aileron" : {
                "is_symmetric" : False
            },
            "elevator" : {
                "is_symmetric" : True
            },
            "rudder" : {
                "is_symmetric" : False
            }
        },
        "airfoils" : "test/airfoils_for_testing.json",
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "semispan" : 4.0,
                "chord" : 1.0,
                "airfoil" : "NACA_0010",
                #"sweep" : 45.0,
                #"dihedral" : [[0.0, 0.0],
                #              [1.0, 30.0]],
                #"twist" : [[0.0, 0.0],
                #           [1.0, 30.0]],
                #"ac_offset" : "kuchemann",
                "control_surface" : {
                    "chord_fraction" : 0.1,
                    "root_span" : 0.1,
                    "tip_span" : 0.9,
                    "control_mixing" : {
                        "aileron" : 1.0
                    }
                },
                "grid" : {
                    "N" : 10,
                    "reid_corrections" : True,
                    "joint_length" : joint_length
                }
            },
            "h_stab" : {
                "ID" : 2,
                "side" : "both",
                "is_main" : False,
                "connect_to" : {
                    "ID" : 1,
                    "location" : "root",
                    "dx" : -4.0
                },
                "semispan" : 2.0,
                "airfoil" : "NACA_0010",
                #"sweep" : 45.0,
                "twist" : -3.95,
                #"ac_offset" : "kuchemann",
                "control_surface" : {
                    "chord_fraction" : 0.5,
                    "control_mixing" : {
                        "elevator" : 1.0
                    }
                },
                "grid" : {
                    "N" : 10,
                    "reid_corrections" : True,
                    "joint_length" : joint_length
                }
            },
            "v_stab" : {
                "ID" : 3,
                "side" : "right",
                "is_main" : False,
                "connect_to" : {
                    "ID" : 1,
                    "location" : "root",
                    "dx" : -4.0,
                    "dz" : -0.1
                },
                "semispan" : 2.0,
                "dihedral" : 90.0,
                "airfoil" : "NACA_0010",
                "control_surface" : {
                    "chord_fraction" : 0.5,
                    "control_mixing" : {
                        "rudder" : 1.0
                    }
                },
                "grid" : {
                    "N" : 10,
                    "reid_corrections" : True,
                    "joint_length" : joint_length
                }
            }
        }
    }

    # Specify state
    state = {
        "velocity" : [20, "mph"],
        "alpha" : 10.0,
        "beta" : 10.0
    }
    control_state = {
        "elevator" : 0.0
    }

    # Load scene with Jackson's corrections
    reid_scene = MX.Scene(input_dict)
    reid_scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)

    # Load scene without Jackson's corrections
    for key, value in airplane_dict["wings"].items():
        value["grid"]["reid_corrections"] = False

    orig_scene = MX.Scene(input_dict)
    orig_scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)

    ## Show wireframes
    #reid_scene.display_wireframe(show_vortices=True)
    #orig_scene.display_wireframe(show_vortices=True)

    # Solve forces
    FM = reid_scene.solve_forces(non_dimensional=False, verbose=True)
    print(json.dumps(FM["plane"]["total"], indent=4))
    FM = orig_scene.solve_forces(non_dimensional=False, verbose=True)
    print(json.dumps(FM["plane"]["total"], indent=4))

    ## Compare
    #val0 = reid_scene._P0#[:,:,2]
    #val1 = reid_scene._P0_joint#[:,:,2]
    #print(val0)
    #print(val1)
    #print((val0-val1)[np.where(np.abs(val0-val1)>1e-10)])