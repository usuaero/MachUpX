# This script is for me to test the functionality of whatever I'm working on at the moment.
import machupX as MX
import json
import numpy as np
import subprocess as sp
import matplotlib.pyplot as plt

if __name__=="__main__":
    
    # Specify input
    input_dict = {
        "solver" : {
            "type" : "nonlinear"
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
        "units" : "English",
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
            "NACA_2410" : {
                "CLa" : 6.28,
                "geometry" : {
                    "NACA" : "2410"
                }
            }
        },
        "plot_lacs" : False,
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "connect_to" : {
                    "y_offset" : 1.0
                },
                "side" : "both",
                "is_main" : True,
                "airfoil" : [[0.0, "NACA_2410"],
                             [1.0, "NACA_2410"]],
                "semispan" : 10.0,
                "dihedral" : [[0.0, 0.0],
                              [1.0, 0.0]],
                "chord" : [[0.0, 1.0],
                           [1.0, 1.0]],
                "sweep" : [[0.0, 30.0],
                           [1.0, 30.0]],
                "grid" : {
                    "N" : 30
                }
            },
            "winglet" : {
                "ID" : 2,
                "side" : "both",
                "connect_to" : {
                    "ID" : 1,
                    "location" : "tip"
                },
                "is_main" : True,
                "airfoil" : [[0.0, "NACA_2410"],
                             [1.0, "NACA_2410"]],
                "semispan" : 10.0,
                "dihedral" : [[0.0, 0.0],
                              [1.0, 0.0]],
                "chord" : [[0.0, 1.0],
                           [1.0, 1.0]],
                "sweep" : [[0.0, 0.0],
                           [1.0, 0.0]],
                "grid" : {
                    "N" : 30
                }
            },
            "h_stab" : {
                "ID" : 3,
                "connect_to" : {
                    "ID" : 1,
                    "location" : "root",
                    "dx" : -5.0,
                    "dz" : -0.5
                },
                "side" : "both",
                "is_main" : False,
                "airfoil" : [[0.0, "NACA_2410"],
                             [1.0, "NACA_2410"]],
                "semispan" : 4.0,
                "dihedral" : [[0.0, 0.0],
                              [1.0, 0.0]],
                "chord" : [[0.0, 1.0],
                           [1.0, 1.0]],
                "sweep" : [[0.0, 30.0],
                           [1.0, 30.0]],
                "grid" : {
                    "N" : 30
                }
            },
            "h_stablet" : {
                "ID" : 4,
                "connect_to" : {
                    "ID" : 3,
                    "location" : "tip"
                },
                "side" : "both",
                "is_main" : False,
                "airfoil" : [[0.0, "NACA_2410"],
                             [1.0, "NACA_2410"]],
                "semispan" : 4.0,
                "dihedral" : [[0.0, 0.0],
                              [1.0, 0.0]],
                "chord" : [[0.0, 1.0],
                           [1.0, 1.0]],
                "sweep" : [[0.0, 0.0],
                           [1.0, 0.0]],
                "grid" : {
                    "N" : 30
                }
            },
            "v_stab" : {
                "ID" : 5,
                "connect_to" : {
                    "ID" : 3,
                    "location" : "root"
                },
                "side" : "right",
                "is_main" : False,
                "airfoil" : [[0.0, "NACA_2410"],
                             [1.0, "NACA_2410"]],
                "semispan" : 4.0,
                "dihedral" : [[0.0, 90.0],
                              [1.0, 90.0]],
                "chord" : [[0.0, 1.0],
                           [1.0, 1.0]],
                "sweep" : [[0.0, 30.0],
                           [1.0, 30.0]],
                "grid" : {
                    "N" : 30
                }
            },
            "T_stab" : {
                "ID" : 6,
                "connect_to" : {
                    "ID" : 5,
                    "location" : "tip",
                    "dz" : -0.1
                },
                "side" : "both",
                "is_main" : False,
                "airfoil" : [[0.0, "NACA_2410"],
                             [1.0, "NACA_2410"]],
                "semispan" : 4.0,
                "dihedral" : [[0.0, 0.0],
                              [1.0, 0.0]],
                "chord" : [[0.0, 1.0],
                           [1.0, 1.0]],
                "sweep" : [[0.0, 30.0],
                           [1.0, 30.0]],
                "grid" : {
                    "N" : 30
                }
            }
        }
    }

    # Specify state
    state = {
        "position" : [0.0, 0.0, 0.0],
        "velocity" : 10.0,
        "alpha" : 5.0,
        "orientation" : [0.0, 0.0, 0.0]
    }
    control_state = {
    }

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)
    scene.display_wireframe()
    FM = scene.solve_forces(verbose=True)
    print(json.dumps(FM["plane"]["total"], indent=4))
    #state["alpha"] = 5.5
    #scene.set_aircraft_state(state=state)
    #FM = scene.solve_forces(initial_guess='previous', verbose=True)
    #print(json.dumps(FM["plane"]["total"], indent=4))
    #state["alpha"] = 0.0
    #scene.set_aircraft_state(state=state)
    #FM = scene.solve_forces(initial_guess='previous', verbose=True)
    #print(json.dumps(FM["plane"]["total"], indent=4))