import machupX as MUX
import numpy as np
import matplotlib.pyplot as plt
import json

np.set_printoptions(linewidth=np.inf)

if __name__=="__main__":

    # Load Jackson's distribution
    with open("test/test_circ_dist.txt", 'r') as jackson_file:
        gamma_jackson_no_sideslip = np.genfromtxt(jackson_file)

    with open("test/circ.dat", 'r') as jackson_file:
        gamma_jackson_sideslip = np.genfromtxt(jackson_file)

    # Parameters
    input_dict = {
        "solver" : {
            "type" : "scipy_fsolve",
            "convergence" : 1e-1
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
                "ac_offset" : "kuchemann",
                "grid" : {
                    "N" : 80,
                    "reid_corrections" : True,
                    "joint_length" : 0.15,
                    "blending_distance" : 0.25
                }
            }
        }
    }
    include_sideslip = 1
    if include_sideslip:
        gamma_jackson = gamma_jackson_sideslip
        state = {
            "velocity" : 1.0,
            "alpha" : 10.0,
            "beta" : 10.0
        }
    else:
        gamma_jackson = gamma_jackson_no_sideslip
        state = {
            "velocity" : 1.0,
            "alpha" : 4.2,
            "beta" : 0.0
        }

    not_grid = 1
    if not_grid:

        # Initialize
        scene = MUX.Scene(input_dict)
        scene.add_aircraft("jackson_wing", airplane_dict, state=state)

        #scene.display_wireframe(show_vortices=True)

        FM = scene.solve_forces(verbose=True, report_by_segment=True, non_dimensional=False)
        print(json.dumps(FM["jackson_wing"]["total"], indent=4))

        plt.figure()
        plt.plot(gamma_jackson[:,0], gamma_jackson[:,1], label='Jackson')
        plt.plot(scene._PC[:,1], scene._gamma, label='MUX')
        plt.legend()
        plt.xlabel("Span Location")
        plt.ylabel("Circulation")
        plt.show()

    else:

        grids = [10, 20, 40, 80, 160]

        plt.figure()
        plt.plot(gamma_jackson[:,0], gamma_jackson[:,1], label='Jackson')
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