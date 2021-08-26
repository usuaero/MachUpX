import os

import machupX as MX
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing as mp

from helpers import get_grayscale_range, richardson_extrap


# Study params
N_sb = 10
blending_distances = np.logspace(-3, 1, N_sb)
N_grids = 10
grids = np.logspace(np.log10(5), np.log10(300), N_grids).astype(int)


def analyze_wing(args):

    sweep, sweep_type, R_A, R_T, c_root = args

    # Announce output
    print("{0:<20}{1:<20}{2:<20}{3:<20}".format(sweep, sweep_type, R_T, R_A))
    
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
    dl_max = np.zeros(N_grids)
    for i, grid in enumerate(grids):

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

            else:
                if j==0:
                    dl_max[i] = scene.get_max_bound_vortex_length()/(b)#*np.cos(np.radians(sweep)))

            finally:
                del scene

    return CL, dl_max


if __name__=="__main__":

    # Create plot directory
    if not os.path.exists("dev/linear_sweep_blending_distance_plots/"):
        os.mkdir("dev/linear_sweep_blending_distance_plots")

    # Options
    sweeps = [-25.0, -15.0, -5.0, 5.0, 15.0, 25.0]
    aspect_ratios = [4.0, 8.0, 12.0, 16.0]
    taper_ratios = [0.0, 0.25, 0.5, 0.75, 1.0]
    c_root = 1.0

    # Create args list
    args_list = []
    for sweep in sweeps[:1]:
        for R_T in taper_ratios[:1]:
            for R_A in aspect_ratios[:1]:
                args_list.append((sweep, 'linear', R_A, R_T, c_root))

    # Run cases
    with mp.Pool(processes=8) as pool:
        results = pool.map(analyze_wing, args_list)

    # Plot formatters
    grid_colors = get_grayscale_range(N_grids, 0, 180)
    sweep_colors = get_grayscale_range(len(sweeps), 0, 180)
    blending_colors = get_grayscale_range(len(blending_distances), 0, 180)
    markers = ['o', 's', 'p', 'P', 'v', '^', '*', 'X', 'x', '+', 'D']

    # Loop through options
    plt.figure()
    for i, result in enumerate(results):
        CL, dl_max = result

        # Loop through blending distances and grids
        for j, blending_distance in enumerate(blending_distances):

            # Process data
            R_bd = blending_distance/dl_max

            # Plot
            #plt.plot(R_bd, CL[:,j], linewidth=0.75, color=sweep_colors[sweeps.index(args_list[i][0])])
            plt.plot(R_bd, CL[:,j], linewidth=0.75, color=blending_colors[j])

    # Format plot
    plt.xlabel('$\\frac{\\Delta s_b b \cos\Lambda}{max\\left(\\left|d\\mathbf{l}\\right|\\right)}$')
    #plt.xlabel('$\\frac{\\Delta s_b}{N}$')
    plt.xscale('log')
    plt.ylabel('$C_L$')
    #plt.legend(title="$\\Delta s_b$")
    plt.show()