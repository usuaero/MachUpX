import machupX as mx
import json
import numpy as np
import matplotlib.pyplot as plt

if __name__=="__main__":
    
    # Specify input
    input_dict = {
        "solver" : {
            "type" : "nonlinear",
            "use_total_velocity" : True,
            "use_swept_sections" : True,
            "use_in_plane" : True,
            "match_machup_pro" : False
        },
        "units" : "English",
        "scene" : {
        }
    }

    # Specify airplane
    airplane_dict = {
        "weight" : 50.0,
        "units" : "English",
        "airfoils" : {
            "NACA_0010" : "dev/NACA_0010.json"
        },
        "plot_lacs" : False,
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "airfoil" : "NACA_0010",
                "semispan" : 4.0,
                #"sweep" : 45.0,
                #"dihedral" : 5.0,
                "dihedral" : [[0.0, 15.0],
                              [0.5, 15.0],
                              [0.5, 15.0],
                              [1.0, 15.0]],
                "sweep" : [[0.0, 45.0],
                           [0.5, 45.0],
                           [0.5, 45.0],
                           [1.0, 45.0]],
                "ac_offset" : "kuchemann",
                "grid" : {
                    "N" : 160,
                    "reid_corrections" : True,
                    "joint_length" : 0.15,
                    "blending_distance" : 0.25
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

    # Initialize plot
    fig, (ax0, ax1) = plt.subplots(ncols=2)

    # Run default case
    scene = mx.Scene(scene_input=input_dict)
    scene.add_aircraft("wing", airplane_dict, state=state)
    dist = scene.distributions()
    y_control = dist["wing"]["main_wing_right"]["cpy"]
    gamma_control = dist["wing"]["main_wing_right"]["circ"]
    w_control = dist["wing"]["main_wing_right"]["w"]
    ax0.plot(y_control, gamma_control, label='Orig')
    ax1.plot(y_control, w_control)

    # Run without corrections
    scene = mx.Scene(scene_input=input_dict)
    airplane_dict["wings"]["main_wing"]["grid"]["reid_corrections"] = False
    scene.add_aircraft("wing", airplane_dict, state=state)
    airplane_dict["wings"]["main_wing"]["grid"]["reid_corrections"] = True
    dist = scene.distributions()
    y_control = dist["wing"]["main_wing_right"]["cpy"]
    gamma_control = dist["wing"]["main_wing_right"]["circ"]
    w_control = dist["wing"]["main_wing_right"]["w"]
    ax0.plot(y_control, gamma_control, label='No Reid')
    ax1.plot(y_control, w_control)

    # Run without Kuchemann
    scene = mx.Scene(scene_input=input_dict)
    airplane_dict["wings"]["main_wing"]["ac_offset"] = 0.0
    scene.add_aircraft("wing", airplane_dict, state=state)
    airplane_dict["wings"]["main_wing"]["ac_offset"] = "kuchemann"
    dist = scene.distributions()
    y_control = dist["wing"]["main_wing_right"]["cpy"]
    gamma_control = dist["wing"]["main_wing_right"]["circ"]
    w_control = dist["wing"]["main_wing_right"]["w"]
    ax0.plot(y_control, gamma_control, label='No Kuchemann')
    ax1.plot(y_control, w_control)

    # Run without swept sections
    input_dict["solver"]["use_swept_sections"] = False
    scene = mx.Scene(scene_input=input_dict)
    input_dict["solver"]["use_swept_sections"] = True
    scene.add_aircraft("wing", airplane_dict, state=state)
    dist = scene.distributions()
    y_control = dist["wing"]["main_wing_right"]["cpy"]
    gamma_control = dist["wing"]["main_wing_right"]["circ"]
    w_control = dist["wing"]["main_wing_right"]["w"]
    ax0.plot(y_control, gamma_control, label='No section corrections')
    ax1.plot(y_control, w_control)

    # Run with shorter joints
    scene = mx.Scene(scene_input=input_dict)
    airplane_dict["wings"]["main_wing"]["grid"]["joint_length"] = 0.05
    scene.add_aircraft("wing", airplane_dict, state=state)
    airplane_dict["wings"]["main_wing"]["grid"]["joint_length"] = 0.15
    dist = scene.distributions()
    y_control = dist["wing"]["main_wing_right"]["cpy"]
    gamma_control = dist["wing"]["main_wing_right"]["circ"]
    w_control = dist["wing"]["main_wing_right"]["w"]
    ax0.plot(y_control, gamma_control, label='Shorter joints')
    ax1.plot(y_control, w_control)

    # Run with longer joints
    scene = mx.Scene(scene_input=input_dict)
    airplane_dict["wings"]["main_wing"]["grid"]["joint_length"] = 0.25
    scene.add_aircraft("wing", airplane_dict, state=state)
    airplane_dict["wings"]["main_wing"]["grid"]["joint_length"] = 0.15
    dist = scene.distributions()
    y_control = dist["wing"]["main_wing_right"]["cpy"]
    gamma_control = dist["wing"]["main_wing_right"]["circ"]
    w_control = dist["wing"]["main_wing_right"]["w"]
    ax0.plot(y_control, gamma_control, label='Longer joints')
    ax1.plot(y_control, w_control)

    ax0.legend()
    ax0.set_title("Circulation")
    ax1.set_title("Downwash")
    plt.show()

    # Convergence study
    fig, (ax0, ax1) = plt.subplots(ncols=2)
    grids = [20, 40, 80, 160, 320, 640, 1280]
    for grid in grids:
        scene = mx.Scene(scene_input=input_dict)
        airplane_dict["wings"]["main_wing"]["grid"]["N"] = grid
        airplane_dict["wings"]["main_wing"]["ac_offset"] = 0.0
        scene.add_aircraft("wing", airplane_dict, state=state)
        dist = scene.distributions()
        y_control = dist["wing"]["main_wing_right"]["cpy"]
        gamma_control = dist["wing"]["main_wing_right"]["circ"]
        w_control = dist["wing"]["main_wing_right"]["w"]
        ax0.plot(y_control, gamma_control, label=str(grid))
        ax1.plot(y_control, w_control)

    ax0.legend()
    ax0.set_title("Circulation")
    ax1.set_title("Downwash")
    plt.show()
    scene.out_gamma()