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
            "type" : "scipy_fsolve"
            #"type" : "nonlinear"
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
        },
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "semispan" : 4.0,
                "airfoil" : "NACA_0010",
                "chord" : [[0.0, 2.0],
                           [0.2, 1.0],
                           [1.0, 0.5]],
                "sweep" : [[0.0, 0.0],
                           [0.5, 20.0]],
                "dihedral" : 10.0,
                "control_surface" : {
                    "chord_fraction" : 0.1,
                    "root_span" : 0.55,
                    "tip_span" : 0.95,
                    "control_mixing" : {
                        "aileron" : 1.0,
                        "elevator" : 1.0
                    }
                },
                "grid" : {
                    "N" : 20,
                    "wing_ID" : 0,
                    "reid_corrections" : True
                }
            },
            "winglets" : {
                "ID" : 2,
                "side" : "both",
                "is_main" : True,
                "connect_to" : {
                    "ID" : 1,
                    "location" : "tip"
                },
                "semispan" : 0.5,
                "dihedral" : [[0.0, 10.0],
                              [0.1, 90.0]],
                "sweep" : 10.0,
                "chord" : [[0.0, 0.5],
                           [1.0, 0.2]],
                "airfoil" : "NACA_0010",
                "grid" : {
                    "N" : 20,
                    "wing_ID" : 0
                }
            }
        }
    }

    # Specify state
    state = {
        "velocity" : 100.0,
        "alpha" : 5.0,
        "beta" : 0.0
    }
    control_state = {
        "elevator" : 0.0,
        "aileron" : 0.0,
        "rudder" : 0.0
    }

    # Load scene with Jackson's corrections
    scene = MX.Scene(input_dict)
    scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)

    #scene.display_wireframe(show_vortices=True)

    ## Solve forces
    #FM = scene.solve_forces(non_dimensional=False, verbose=True)
    #print(json.dumps(FM["plane"]["total"], indent=4))

    ## Get derivatives
    #derivs = scene.aircraft_derivatives(wind_frame=False)
    #print(json.dumps(derivs["plane"], indent=4))

    grids = [10, 20, 40, 80, 160]

    plt.figure()
    for grid in grids:
        airplane_dict["wings"]["main_wing"]["grid"]["N"] = grid
        scene = MX.Scene(input_dict)
        scene.add_aircraft("jackson_wing", airplane_dict, state=state)
        scene.solve_forces(verbose=True)
        plt.plot(scene._PC[:,1], scene._gamma, label='MUX {0}'.format(grid))
    plt.legend()
    plt.xlabel("Span Location")
    plt.ylabel("Circulation")
    plt.show()