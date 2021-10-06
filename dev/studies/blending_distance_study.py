# This script is for me to easily test convergences
import os

import machupX as MX
import numpy as np
import math as m
import matplotlib.pyplot as plt
import multiprocessing as mp

from helpers import get_grayscale_range, richardson_extrap
from mpl_toolkits import mplot3d


def analyze_wing(args):

    sweep, sweep_type, R_A, R_T, c_root = args

    # Announce output
    print("{0:<20}{1:<20}{2:<20}{3:<20}".format(sweep, sweep_type, R_T, R_A))

    # Study params
    N_sb = 20
    blending_distances = np.logspace(-2, 1, N_sb)
    N_grids = 10
    grids = np.logspace(1, np.log10(200), N_grids).astype(int)

    # Color ranges
    grid_colors = get_grayscale_range(N_grids, 0, 180)
    blending_distance_colors = get_grayscale_range(N_sb, 0, 180)
    
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

    # Loop through blending distances and grids
    CL = np.zeros((N_grids, N_sb))
    p = np.zeros(N_sb)
    plt.figure()
    for i, grid in enumerate(grids):


        for j, blending_distance in enumerate(blending_distances):

            # Update params
            if sweep_type == "linear":
                wing_dict["wings"]["main_wing"]["sweep"] = [[0.0, 0.0],
                                                            [1.0, sweep]]
            else:
                wing_dict["wings"]["main_wing"]["sweep"] = [[0.0, sweep],
                                                            [1.0, sweep]]

            wing_dict["wings"]["main_wing"]["semispan"] = 0.25*R_A*c_root*(1.0+R_T)
            wing_dict["wings"]["main_wing"]["chord"] = [[0.0, c_root], [1.0, R_T*c_root]]
            wing_dict["wings"]["main_wing"]["grid"]["N"] = grid
            wing_dict["wings"]["main_wing"]["grid"]["blending_distance"] = blending_distance

            # Load scene
            scene = MX.Scene(input_dict)
            scene.add_aircraft("wing", wing_dict, state=state)
            try:
                FM = scene.solve_forces()
                CL[i,j] = FM["wing"]["total"]["CL"]
            except:
                CL[i,j] = np.nan
            del scene

        plt.plot(blending_distances, CL[i,:], 'ko', label=str(grid), markersize=150/grid, mfc='none', linewidth=0.5)

    plt.xscale('log')
    plt.xlabel('$\\Delta s_b$')
    plt.ylabel('$C_L$')
    plt.legend(title="N")
    plt.savefig("dev/studies/blending_distance_plots_marker_size/CL_{0}_{1}_sweep_RA_{2}_RT_{3}.pdf".format(sweep, sweep_type, R_A, R_T))
    plt.close()

    # Calculate Richardson extrapolations
    CL_extrap = np.zeros(N_sb)
    plt.figure()
    for i, blending_distance in enumerate(blending_distances):
        CL_extrap[i],p[i] = richardson_extrap(grids, CL[:,i])
        CL_err = np.abs(CL[:,i]-CL_extrap[i])/np.abs(CL_extrap[i])
        plt.plot(grids, CL_err, 'o-', label=str(blending_distance), color=blending_distance_colors[i], markersize=3)

    plt.xscale('log')
    plt.yscale('log')
    plt.xlabel('$N$')
    plt.ylabel('Error in $C_L$')
    #plt.legend(title="$\\Delta s_b$")
    plt.savefig("dev/studies/blending_distance_plots_marker_size/error_{0}_{1}_sweep_RA_{2}_RT_{3}.pdf".format(sweep, sweep_type, R_A, R_T))
    plt.close()

    return p


if __name__=="__main__":

    # Create plot directory
    if not os.path.exists("dev/studies/blending_distance_plots_marker_size/"):
        os.mkdir("dev/studies/blending_distance_plots_marker_size")

    # Options
    sweeps = [-25.0, -15.0, -5.0, 5.0, 15.0, 25.0]
    sweep_types = ['linear', 'constant']
    taper_ratios = [1.0, 0.75, 0.5, 0.25, 0.0]
    aspect_ratios = [4.0, 8.0, 12.0, 16.0]
    c_root = 1.0

    # Create args list
    args_list = []
    for sweep in sweeps:
        for sweep_type in sweep_types:
            for R_T in taper_ratios:
                for R_A in aspect_ratios:
                    args_list.append((sweep, sweep_type, R_A, R_T, c_root))


    print("{0:<20}{1:<20}{2:<20}{3:<20}".format("Sweep [deg]", "Sweep Type", "Taper Ratio", "Aspect Ratio"))
    print("".join(["-"]*80))

    # Send to pool
    with mp.Pool() as pool:
        convergence_rates = pool.map(analyze_wing, args_list)

    # Write convergence rates to file
    N_sb = 20
    blending_distances = np.logspace(-2, 1, N_sb)
    with open("dev/blending_distance_plots/convergence_rates.csv", 'w') as file_handle:
        print("sweep,type,R_A,R_T,"+",".join([str(s) for s in blending_distances]), file=file_handle)

        for args, rates in zip(args_list, convergence_rates):
            print("{0},{1},{2},{3},".format(*args)+",".join([str(p) for p in rates]), file=file_handle)
