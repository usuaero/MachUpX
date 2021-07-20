# This script is for me to easily test convergences
import machupX as MX
import pypan as pp
import json
import os
import numpy as np
import subprocess as sp
import matplotlib.pyplot as plt
from stl import mesh
from mpl_toolkits import mplot3d


def get_grayscale_range(N, min, max):
    colors_int = np.linspace(min, max, N_grids).astype(int)[::-1]
    colors = []
    for color_int in colors_int:
        hex_code = hex(color_int).replace("0x", "")
        if len(hex_code) == 1:
            hex_code = "0"+hex_code
        color = "#"+"".join([hex_code]*3)
        colors.append(color)

    return colors


if __name__=="__main__":

    # Create plot directory
    if not os.path.exists("dev/linear_sweep_blending_distance_plots/"):
        os.mkdir("dev/linear_sweep_blending_distance_plots")
    
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

    # Specify wing
    wing_dict = {
        "weight" : 50.0,
        "units" : "English",
        "plot_lacs" : False,
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "semispan" : 10.0,
                "dihedral" : [[0.0, 0.0],
                              [1.0, 0.0]],
                "chord" : [[0.0, 1.0],
                           [1.0, 1.0]],
                "sweep" : [[0.0, 0.0],
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

    # Options
    sweeps = [-25.0, -15.0, -5.0, 5.0, 15.0, 25.0]
    wingspans = [10.0]
    blending_distances = np.logspace(-3, 1, 10)
    N_grids = 15
    grids = np.logspace(np.log10(4), np.log10(400), N_grids).astype(int)

    # Plot formatters
    grid_colors = get_grayscale_range(N_grids, 0, 180)
    sweep_colors = get_grayscale_range(len(sweeps), 0, 180)
    blending_colors = get_grayscale_range(len(blending_distances), 0, 180)
    markers = ['o', 's', 'p', 'P', 'v', '^', '*', 'X', 'x', '+', 'D']

    # Loop through options
    plt.figure()
    for i, sweep in enumerate(sweeps):
        for wingspan in wingspans:

            # Loop through blending distances and grids
            for j, blending_distance in enumerate(blending_distances):

                # Initialize storage
                Ds_b = []
                dl_max = []
                CL = []
                N_nodes = []

                for k, grid in enumerate(grids):

                    # Update params
                    wing_dict["wings"]["main_wing"]["sweep"] = [[0.0, 0.0],
                                                                [1.0, sweep]]
                    wing_dict["wings"]["main_wing"]["semispan"] = wingspan*0.5
                    wing_dict["wings"]["main_wing"]["grid"]["N"] = grid
                    wing_dict["wings"]["main_wing"]["grid"]["blending_distance"] = blending_distance

                    # Load scene
                    scene = MX.Scene(input_dict)
                    scene.add_aircraft("wing", wing_dict, state=state)
                    try:
                        FM = scene.solve_forces()

                    except:
                        pass

                    else:
                        Ds_b.append(blending_distance)
                        dl_max.append(scene.get_max_bound_vortex_length()/(wingspan*np.cos(np.radians(sweep))))
                        N_nodes.append(grid)
                        CL.append(FM["wing"]["total"]["CL"])

                    finally:
                        del scene


                # Process data
                Ds_b = np.array(Ds_b)
                dl_max = np.array(dl_max)
                N_nodes = np.array(N_nodes)
                CL = np.array(CL)
                R_bd = Ds_b/dl_max

                # Plot
                #plt.plot(R_bd, CL, '-'+markers[i], color=blending_colors[j])
                plt.plot(R_bd, CL, 'k', linewidth=0.5)

    # Format plot
    plt.xlabel('$\\frac{\\Delta s_b b \cos\Lambda}{max\\left(\\left|d\\mathbf{l}\\right|\\right)}$')
    #plt.xlabel('$\\frac{\\Delta s_b}{N}$')
    plt.xscale('log')
    plt.ylabel('$C_L$')
    #plt.legend(title="$\\Delta s_b$")
    plt.show()