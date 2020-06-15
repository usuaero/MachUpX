"""Compares the total results based on how the control point velocities are used to redimensionalize section lift coefficients."""

import machupX as mx
import json
import numpy as np

if __name__=="__main__":

    # Input
    input_dict = {
        "units" : "English",
        "solver" : {
            "type" : "nonlinear",
            "correct_sections_for_sweep" : True,
            "machup_pro_deriv" : False
        },
        "scene" : {
            "atmosphere" : {
                "density" : 0.0023769
            }
        }
    }

    # Straight wing
    wing_dict = {
        "weight" : 10.0,
        "units" : "English",
        "airfoils" : {
            "NACA_0010" : "dev/NACA_0010.json"
        },
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "is_main" : True,
                "semispan" : 4.0,
                "side" : "both"
            }
        }
    }

    # State
    state = {
        "velocity" : 10.0,
        "alpha" : 5.0
    }

    # Output
    print("---Results---")

    # Total velocity, no sideslip
    input_dict["solver"]["correct_sections_for_sweep"] = False
    scene = mx.Scene(input_dict)
    scene.add_aircraft("wing", wing_dict, state=state)
    FM = scene.solve_forces(non_dimensional=False)
    redim_vels_tot_no_sideslip = scene._V_i_2
    print("No sideslip, jointed vortices, and total velocity.")
    print("    FL = {0}".format(FM["wing"]["total"]["FL"]))

    # Total effective velocity, no sideslip
    input_dict["solver"]["correct_sections_for_sweep"] = True
    scene = mx.Scene(input_dict)
    scene.add_aircraft("wing", wing_dict, state=state)
    FM = scene.solve_forces(non_dimensional=False)
    redim_vels_eff_no_sideslip = scene._V_i_eff_2
    print("No sideslip, jointed vortices, and effective velocity.")
    print("    FL = {0}".format(FM["wing"]["total"]["FL"]))
    print("    ||V_i_2-V_i_eff_2|| = {0}".format(np.linalg.norm(redim_vels_eff_no_sideslip-redim_vels_tot_no_sideslip)))

    # Total velocity, sideslip
    input_dict["solver"]["correct_sections_for_sweep"] = False
    scene = mx.Scene(input_dict)
    state["beta"] = 10.0
    scene.add_aircraft("wing", wing_dict, state=state)
    FM = scene.solve_forces(non_dimensional=False)
    redim_vels_tot_no_sideslip = scene._V_i_2
    print("Sideslip, jointed vortices, and total velocity.")
    print("    FL = {0}".format(FM["wing"]["total"]["FL"]))

    # Total effective velocity, sideslip
    input_dict["solver"]["correct_sections_for_sweep"] = True
    scene = mx.Scene(input_dict)
    state["beta"] = 10.0
    scene.add_aircraft("wing", wing_dict, state=state)
    FM = scene.solve_forces(non_dimensional=False)
    redim_vels_eff_no_sideslip = scene._V_i_eff_2
    print("Sideslip, jointed vortices, and effective velocity.")
    print("    FL = {0}".format(FM["wing"]["total"]["FL"]))
    print("    ||V_i_2-V_i_eff_2|| = {0}".format(np.linalg.norm(redim_vels_eff_no_sideslip-redim_vels_tot_no_sideslip)))

    # Total velocity, no sideslip
    input_dict["solver"]["correct_sections_for_sweep"] = False
    scene = mx.Scene(input_dict)
    state["beta"] = 0.0
    wing_dict["wings"]["main_wing"]["grid"] = {"reid_corrections" : False}
    scene.add_aircraft("wing", wing_dict, state=state)
    FM = scene.solve_forces(non_dimensional=False)
    redim_vels_tot_no_sideslip = scene._V_i_2
    print("No sideslip, no jointed vortices, and total velocity.")
    print("    FL = {0}".format(FM["wing"]["total"]["FL"]))

    # Total effective velocity, no sideslip
    input_dict["solver"]["correct_sections_for_sweep"] = True
    scene = mx.Scene(input_dict)
    state["beta"] = 0.0
    wing_dict["wings"]["main_wing"]["grid"] = {"reid_corrections" : False}
    scene.add_aircraft("wing", wing_dict, state=state)
    FM = scene.solve_forces(non_dimensional=False)
    redim_vels_eff_no_sideslip = scene._V_i_eff_2
    print("No sideslip, no jointed vortices, and effective velocity.")
    print("    FL = {0}".format(FM["wing"]["total"]["FL"]))
    print("    ||V_i_2-V_i_eff_2|| = {0}".format(np.linalg.norm(redim_vels_eff_no_sideslip-redim_vels_tot_no_sideslip)))