# This script is for me to easily test convergences
import os

import machupX as MX
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing as mp

from helpers import get_grayscale_range, richardson_extrap
from mpl_toolkits import mplot3d


if __name__=="__main__":

    # Options
    sweep = 25.0
    sweep_types = ['linear', 'constant']
    R_T = 0.0
    R_A = 8.0
    c_root = 1.0
    grid_N = 200

    # Study params
    N_sb = 10
    blending_distances = np.logspace(-1, 0, N_sb)

    # Color ranges
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

    for sweep_type in sweep_types:

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
                        "N" : grid_N,
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

        # Loop through blending distances
        gamma_i = np.zeros((N_sb, 2*grid_N))
        plt.figure()
        for j, blending_distance in enumerate(blending_distances):

            # Update params
            if sweep_type == "linear":
                wing_dict["wings"]["main_wing"]["sweep"] = [[0.0, 0.0],
                                                            [1.0, sweep]]
            else:
                wing_dict["wings"]["main_wing"]["sweep"] = [[0.0, sweep],
                                                            [1.0, sweep]]

            b = 0.25*R_A*c_root*(1.0+R_T)
            wing_dict["wings"]["main_wing"]["semispan"] = b
            wing_dict["wings"]["main_wing"]["chord"] = [[0.0, c_root], [1.0, R_T*c_root]]
            wing_dict["wings"]["main_wing"]["grid"]["blending_distance"] = blending_distance

            # Load scene
            scene = MX.Scene(input_dict)
            scene.add_aircraft("wing", wing_dict, state=state)
            try:
                FM = scene.solve_forces(verbose=True)
                gamma_i[j,:] = np.copy(scene._gamma)
            except:
                gamma_i[j,:] = np.nan

            plt.plot(scene._PC[:,1]/b, gamma_i[j,:], color=blending_distance_colors[j])

            del scene

        plt.xlabel('$y/b$')
        plt.ylabel('$\\Gamma$')
        plt.savefig("dev/blending_distance_plots/gamma_dist_{0}_{1}_sweep_RA_{2}_RT_{3}.pdf".format(sweep, sweep_type, R_A, R_T))
        plt.close()