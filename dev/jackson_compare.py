import machupX as MUX
import numpy as np
import matplotlib.pyplot as plt
import json

np.set_printoptions(linewidth=np.inf)

if __name__=="__main__":

    # Parameters
    input_dict = {
        "solver" : {
            "type" : "nonlinear"
        },
        "units" : "English",
        "scene" : {}
    }
    airplane_dict = {
        "CG" : [0,0,0],
        "weight" : 10.0,
        "airfoils" : {
            "NACA_0012" : {
                "CLa" : 6.907213339669221,
                "geometry" : {
                    "NACA" : "0012"
                }
            }
        },
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "semispan" : 2.5,
                "chord" : 1.0,
                "airfoil" : "NACA_0012",
                "sweep" : 45.0,
                "ll_offset" : "kuchemann",
                "grid" : {
                    "N" : 80,
                    "reid_corrections" : True,
                    "joint_length" : 0.15,
                    "blending_distance" : 1.0
                }
            }
        }
    }

    # Select comparison case
    case_no = 1
    if case_no == 0:
        with open("dev/jackson_data/10_a_10_b_circ.dat", 'r') as jackson_file:
            gamma_jackson = np.genfromtxt(jackson_file)
        state = {
            "velocity" : 1.0,
            "alpha" : 10.0,
            "beta" : 10.0
        }
    elif case_no == 1:
        with open("dev/jackson_data/4.2_a_0_b_circ.dat", 'r') as jackson_file:
            gamma_jackson = np.genfromtxt(jackson_file)
        state = {
            "velocity" : 1.0,
            "alpha" : 4.2,
            "beta" : 0.0
        }
    elif case_no == 2:
        with open("dev/jackson_data/10_a_0_b_circ.dat", 'r') as jackson_file:
            gamma_jackson = np.genfromtxt(jackson_file)
        state = {
            "velocity" : 1.0,
            "alpha" : 10,
            "beta" : 0.0
        }
    elif case_no == 3:
        with open("dev/jackson_data/15_a_15_b_circ.dat", 'r') as jackson_file:
            gamma_jackson = np.genfromtxt(jackson_file)
        state = {
            "velocity" : 1.0,
            "alpha" : 15.0,
            "beta" : 15.0
        }

    not_grid = 0
    if not_grid:

        # Initialize
        scene = MUX.Scene(input_dict)
        scene.add_aircraft("jackson_wing", airplane_dict, state=state)

        #scene.display_wireframe(show_vortices=True)

        FM = scene.solve_forces(verbose=True, report_by_segment=True, non_dimensional=False)
        print(json.dumps(FM["jackson_wing"]["total"], indent=4))

        plt.figure()
        plt.plot(gamma_jackson[::-1,0], gamma_jackson[:,1], label='Jackson')
        plt.plot(scene._PC[:,1], scene._gamma, label='MUX')
        plt.legend()
        plt.xlabel("Span Location")
        plt.ylabel("Circulation")
        plt.show()

    else:

        grids = [10, 20, 40, 80, 160]

        plt.figure()
        plt.plot(gamma_jackson[::-1,0], gamma_jackson[:,1], label='Jackson')
        for grid in grids:
            airplane_dict["wings"]["main_wing"]["grid"]["N"] = grid
            scene = MUX.Scene(input_dict)
            scene.add_aircraft("jackson_wing", airplane_dict, state=state)
            scene.solve_forces(verbose=True)
            plt.plot(scene._PC[:,1], scene._gamma, label='MUX {0}'.format(grid))
        plt.legend()
        plt.xlabel("Span Location")
        plt.ylabel("Circulation")
        plt.show()