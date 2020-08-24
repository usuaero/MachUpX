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
            "use_total_velocity" : True
        },
        "units" : "English",
        "scene" : {
            "atmosphere" : {
            }
        }
    }

    # Specify airplane
    airplane_dict = {
        "weight" : 50.0,
        "units" : "SI",
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
            "NACA_4410" : "dev/NACA_4410.json",
            "NACA_0010" : "dev/NACA_0010.json"
        },
        "plot_lacs" : False,
        "wings" : {
            #"winglets" : {
            #    "ID" : 2,
            #    "side" : "both",
            #    "is_main" : True,
            #    "connect_to" : {
            #        "ID" : 1,
            #        "location" : "tip",
            #        "dz" : -0.001
            #    },
            #    "semispan" : 0.5,
            #    "dihedral" : 90.0,
            #    "sweep" : 10.0,
            #    "chord" : [[0.0, 0.5],
            #               [1.0, 0.2]],
            #    "airfoil" : "NACA_0010",
            #    "grid" : {
            #        "N" : 20,
            #        "wing_ID" : 1,
            #        "reid_corrections" : True
            #    }
            #},
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "airfoil" : "NACA_0010",
                "semispan" : 4.0,
                "sweep" : 45.0,
                "dihedral" : [[0.0, 0.0],
                              [1.0, 180.0]],
                "twist" : [[0.0, 0.0],
                           [1.0, 20.0]],
                "ac_offset" : "kuchemann",
                "control_surface" : {
                    "chord_fraction" : 0.4,
                    "root_span" : 0.55,
                    "tip_span" : 0.95,
                    "control_mixing" : {
                        "aileron" : 1.0,
                        "elevator" : 1.0
                    }
                },
                "grid" : {
                    "N" : 80,
                    "wing_ID" : 1,
                    "reid_corrections" : True
                }
            }
        }
    }

    # Specify state
    state = {
        "velocity" : 100.0,
        "alpha" : 10.0,
        "beta" : 0.0
    }

    control_state = {
        "elevator" : 0.0,
        "aileron" : 0.0,
        "rudder" : 0.0
    }

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)

    #scene.display_wireframe(show_vortices=False)
    scene.export_dxf()

    ## Solve forces
    #FM = scene.solve_forces(non_dimensional=False, verbose=True)
    #print(json.dumps(FM["plane"]["total"], indent=4))

    #scene.distributions(filename="test.dist")

    ## Get derivatives
    #derivs = scene.derivatives(wind_frame=False)
    #print(json.dumps(derivs["plane"], indent=4))

    ## Get state derivatives
    #derivs = scene.state_derivatives()
    #print(json.dumps(derivs["plane"], indent=4))