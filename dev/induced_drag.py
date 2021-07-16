import json
import copy

import machupX as MX
import numpy as np
import subprocess as sp
import matplotlib.pyplot as plt

from stl import mesh
from mpl_toolkits import mplot3d


def get_induced_drag(input_dict, airplane_dict, state):

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("wing", airplane_dict, state=state)
    S,_,_ = scene.get_aircraft_reference_geometry()

    # Solve forces
    FM = scene.solve_forces(non_dimensional=False, verbose=True)

    # Get induced drag distribution
    dist = scene.distributions()
    s = np.array(dist["wing"]["main_wing_right"]["span_frac"])
    CD_i = np.array(dist["wing"]["main_wing_right"]["CD_i"])

    return s, CD_i


if __name__=="__main__":
    
    # Specify input
    input_dict = {
        "solver" : {
            "type" : "nonlinear",
            "use_swept_sections" : True,
            "use_total_velocity" : True,
            "use_in_plane" : True,
            "match_machup_pro" : False
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
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "semispan" : 6.0,
                "sweep" : 45.0,
                "grid" : {
                    "N" : 100,
                    "reid_corrections" : True
                }
            }
        }
    }

    # Specify state
    state = {
        "velocity" : 100.0,
        "alpha" : 5.0
    }

    # Original case
    s, CD_i_orig = get_induced_drag(input_dict, airplane_dict, state)

    # Turn off Reid corrections
    new_airplane_dict = copy.deepcopy(airplane_dict)
    new_airplane_dict["wings"]["main_wing"]["grid"]["reid_corrections"] = False
    s, CD_i_no_reid = get_induced_drag(input_dict, new_airplane_dict, state)

    # Turn off swept sections
    new_input_dict = copy.deepcopy(input_dict)
    new_input_dict["solver"]["use_swept_sections"] = False
    s, CD_i_no_swept = get_induced_drag(new_input_dict, airplane_dict, state)

    # Turn off total velocity
    new_input_dict = copy.deepcopy(input_dict)
    new_input_dict["solver"]["use_total_velocity"] = False
    s, CD_i_no_total_vel = get_induced_drag(new_input_dict, airplane_dict, state)

    # Turn off in-plane velocity
    new_input_dict = copy.deepcopy(input_dict)
    new_input_dict["solver"]["use_in_plane"] = False
    s, CD_i_no_in_plane = get_induced_drag(new_input_dict, airplane_dict, state)

    # Set to only Reid corrections
    new_input_dict = copy.deepcopy(input_dict)
    new_input_dict["solver"]["use_swept_sections"] = False
    new_input_dict["solver"]["use_total_velocity"] = False
    new_input_dict["solver"]["use_in_plane"] = False
    s, CD_i_only_reid = get_induced_drag(new_input_dict, airplane_dict, state)

    # Match MachUp Pro
    new_input_dict["solver"]["match_machup_pro"] = True
    s, CD_i_pro = get_induced_drag(new_input_dict, new_airplane_dict, state)

    # Plot
    plt.figure()
    plt.plot(s, CD_i_orig, 'k-', label='True G-H')
    plt.plot(s, CD_i_no_reid, 'k:', label='No Reid Corrections')
    plt.plot(s, CD_i_no_swept, 'k-.', label='No Swept Sections')
    plt.plot(s, CD_i_no_total_vel, 'ko', label='No Total Velocity')
    plt.plot(s, CD_i_no_in_plane, 'k.', label='No In Plane')
    plt.plot(s, CD_i_only_reid, 'kx', label='Only Reid Corrections')
    plt.plot(s, CD_i_pro, 'k--', label='MachUp Pro')
    plt.legend()
    plt.xlabel('Span Fraction')
    plt.ylabel('CD_i')
    plt.show()