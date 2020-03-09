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
            "convergence" : 1e-10,
            "relaxation" : 0.9
        },
        "units" : "English",
        "scene" : {
            "atmosphere" : {},
            "aircraft" : {}
        }
    }

    # Specify airplane
    airplane_dict = {
        "CG" : [0,0,0],
        "weight" : 100.0,
        "reference" : {
            "area" : 8.0,
            "longitudinal_length" : 1.0,
            "lateral_length" : 4.0
        },
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
                "sweep" : 45.0,
                "ac_offset" : "kuchemann",
                "control_surface" : {
                    "chord_fraction" : 0.1,
                    "control_mixing" : {
                        "aileron" : 1.0
                    }
                },
                "grid" : {
                    "N" : 40
                }
            #},
            #"h_stab" : {
            #    "ID" : 2,
            #    "side" : "both",
            #    "is_main" : False,
            #    "connect_to" : {
            #        "ID" : 1,
            #        "location" : "root",
            #        "dx" : -4.0
            #    },
            #    "semispan" : 2.0,
            #    "airfoil" : "NACA_0010",
            #    "control_surface" : {
            #        "chord_fraction" : 0.5,
            #        "control_mixing" : {
            #            "elevator" : 1.0
            #        }
            #    },
            #    "grid" : {
            #        "N" : 40
            #    }
            #},
            #"v_stab" : {
            #    "ID" : 3,
            #    "side" : "right",
            #    "is_main" : False,
            #    "connect_to" : {
            #        "ID" : 1,
            #        "location" : "root",
            #        "dx" : -4.0,
            #        "dz" : -0.1
            #    },
            #    "semispan" : 2.0,
            #    "dihedral" : 90.0,
            #    "airfoil" : "NACA_0010",
            #    "control_surface" : {
            #        "chord_fraction" : 0.5,
            #        "control_mixing" : {
            #            "rudder" : 1.0
            #        }
            #    },
            #    "grid" : {
            #        "N" : 40
            #    }
            }
        }
    }

    # Specify state
    state = {
        "velocity" : [100, "mph"],
        "alpha" : 2.0
    }

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("plane", airplane_dict, state=state)

    scene.display_wireframe()
    #scene.distributions(make_plots=['cpx'], show_plots=True)

    print("Original state")
    FM = scene.solve_forces(non_dimensional=False, verbose=True)
    print(json.dumps(FM["plane"]["total"], indent=4))

    #trim_angles = scene.aircraft_pitch_trim(verbose=True, set_trim_state=True)
    #print(json.dumps(trim_angles["plane"], indent=4))

    #print("---Trim State---")
    #FM = scene.solve_forces(non_dimensional=False, verbose=True)
    #print(json.dumps(FM["plane"]["total"], indent=4))

    #print("---Derivatives---")
    #derivs = scene.aircraft_derivatives()
    #print(json.dumps(derivs["plane"]["stability"], indent=4))

    #print("---MAC---")
    #MAC = scene.aircraft_mean_aerodynamic_chord()
    #print(json.dumps(MAC["plane"], indent=4))

    #print("---Aerodynamic Center---")
    #AC = scene.aircraft_aero_center()
    #print(json.dumps(AC["plane"], indent=4))
    #scene.remove_aircraft("plane")