# This script is for me to easily test convergences
import machupX as MX
import pypan as pp
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
            "max_iterations" : 1000
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
        "plot_lacs" : False,
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "semispan" : 10.0,
                "chord" : 1.0,
                "sweep" : [[0.0, 30.0],
                           [1.0, 30.0]],
                "grid" : {
                    "N" : 80,
                    "blending_distance" : 0.25
                }
            }
        }
    }

    # Specify state
    state = {
        "velocity" : 10.0,
        "alpha" : 5.0
    }

    # Specify grids
    N_grids = 11
    grids = np.logspace(1, 3, N_grids).astype(int)

    # Grid colors
    grid_colors_int = np.linspace(0, 180, N_grids).astype(int)[::-1]
    grid_colors = []
    for grid_color_int in grid_colors_int:
        hex_code = hex(grid_color_int).replace("0x", "")
        if len(hex_code) == 1:
            hex_code = "0"+hex_code
        color = "#"+"".join([hex_code]*3)
        grid_colors.append(color)

    # Loop through grids
    plt.figure()
    for i, grid in enumerate(grids):

        # Update grid
        airplane_dict["wings"]["main_wing"]["grid"]["N"] = grid

        # Load scene
        scene = MX.Scene(input_dict)
        scene.add_aircraft("wing", airplane_dict, state=state)
        try:
            FM = scene.solve_forces()
        except:
            continue
        dist = scene.distributions()

        plt.plot(dist["wing"]["main_wing_right"]["cpy"], dist["wing"]["main_wing_right"]["circ"], label=str(grid), color=grid_colors[i])

    plt.xlabel('$y$')
    plt.ylabel('$\Gamma$')
    plt.legend()
    plt.show()