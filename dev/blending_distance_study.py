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

if __name__=="__main__":

    # Create plot directory
    if not os.path.exists("dev/blending_distance_plots/"):
        os.mkdir("dev/blending_distance_plots")
    
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
    sweep_types = ['linear', 'constant']
    wingspans = [5.0, 10.0]
    blending_distances = np.logspace(-2, 1, 20)
    N_grids = 10
    grids = np.logspace(1, np.log10(200), N_grids).astype(int)

    # Grid colors
    grid_colors_int = np.linspace(0, 180, N_grids).astype(int)[::-1]
    grid_colors = []
    for grid_color_int in grid_colors_int:
        hex_code = hex(grid_color_int).replace("0x", "")
        if len(hex_code) == 1:
            hex_code = "0"+hex_code
        color = "#"+"".join([hex_code]*3)
        grid_colors.append(color)

    # Loop through options
    print("{0:<20}{1:<20}{2:<20}{3:<20}".format("Sweep [deg]", "Sweep Type", "Wingspan [ft]", "Grid Nodes"))
    print("".join(["-"]*80))
    for sweep in sweeps:
        for sweep_type in sweep_types:
            for wingspan in wingspans:

                # Loop through blending distances and grids
                plt.figure()
                for i, grid in enumerate(grids):

                    print("{0:<20}{1:<20}{2:<20}{3:<20}".format(sweep, sweep_type, wingspan, grid))
                    CL = []

                    for blending_distance in blending_distances:

                        # Update params
                        if sweep_type == "linear":
                            wing_dict["wings"]["main_wing"]["sweep"] = [[0.0, 0.0],
                                                                        [1.0, sweep]]
                        else:
                            wing_dict["wings"]["main_wing"]["sweep"] = [[0.0, sweep],
                                                                        [1.0, sweep]]

                        wing_dict["wings"]["main_wing"]["semispan"] = wingspan*0.5
                        wing_dict["wings"]["main_wing"]["grid"]["N"] = grid
                        wing_dict["wings"]["main_wing"]["grid"]["blending_distance"] = blending_distance

                        # Load scene
                        scene = MX.Scene(input_dict)
                        scene.add_aircraft("wing", wing_dict, state=state)
                        try:
                            FM = scene.solve_forces()
                            CL.append(FM["wing"]["total"]["CL"])
                        except:
                            CL.append(np.nan)
                        del scene

                    plt.plot(blending_distances, CL, 'x-', label=str(grid), color=grid_colors[i])

                #plt.axvline(x=0.25)
                #plt.title("{0} deg {1} sweep, b = {2} ft".format(sweep, sweep_type, wingspan))
                plt.xscale('log')
                plt.xlabel('$\\Delta s_b$')
                plt.ylabel('$C_L$')
                #plt.legend()
                plt.savefig("dev/blending_distance_plots/{0}_{1}_sweep_b_{2}_mux_fixed.pdf".format(sweep, sweep_type, wingspan))
                plt.close()