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
            "type" : "nonlinear",
        },
        "scene" : {
            "atmosphere" : {
            }
        }
    }

    # Specify airplane
    airplane_dict = {
        "CG" : [0.0, 0.0, 0.0],
        "weight" : 50.0,
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
        "airfoils" : {
            "NACA_4410" : "test/NACA_4410.json",
            "NACA_0010" : "test/NACA_0010.json"
        },#"test/airfoils_for_testing.json",
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "semispan" : 4.0,
                "airfoil" : "NACA_4410",
                "sweep" : 45.0,
                "control_surface" : {
                    "chord_fraction" : 0.1,
                    "root_span" : 0.5,
                    "tip_span" : 0.95,
                    "control_mixing" : {
                        "aileron" : 1.0
                    }
                },
                "grid" : {
                    "N" : 40,
                    "reid_corrections" : True
                }
            },
            "h_stab" : {
                "ID" : 2,
                "side" : "both",
                "is_main" : False,
                "connect_to" : {
                    "ID" : 1,
                    "location" : "root",
                    "dx" : -3.0
                },
                "semispan" : 2.0,
                "airfoil" : "NACA_0010",
                "twist" : -3.95,
                "control_surface" : {
                    "chord_fraction" : 0.5,
                    "control_mixing" : {
                        "elevator" : 1.0
                    }
                },
                "grid" : {
                    "N" : 40,
                    "reid_corrections" : True
                }
            },
            "v_stab" : {
                "ID" : 3,
                "side" : "right",
                "is_main" : False,
                "connect_to" : {
                    "ID" : 1,
                    "location" : "root",
                    "dx" : -3.0,
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
                    "N" : 40,
                    "reid_corrections" : True
                }
            }
        }
    }

    # Specify state
    state = {
        "velocity" : 100.0,
        "alpha" : 0.0,
        "beta" : 0.0
    }
    control_state = {
        "elevator" : 0.0,
        "aileron" : 0.0,
        "rudder" : 0.0
    }

    # Load scene with Jackson's corrections
    scene = MX.Scene(input_dict)
    #scene.add_aircraft("plane", "test/mux_airplane.json", state=state, control_state=control_state)
    scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)

    #scene.display_wireframe(show_vortices=False)

    # Solve forces
    FM = scene.solve_forces(non_dimensional=False, verbose=True)
    print(json.dumps(FM["plane"]["total"], indent=4))

    ## Pitch trim
    #pitch_trim = scene.aircraft_pitch_trim(verbose=True)
    #print(json.dumps(pitch_trim, indent=4))

    #scene.export_pylot_model(set_accel_derivs=True, controller_type="keyboard")