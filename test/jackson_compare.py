import machupX as MUX
import numpy as np
import matplotlib.pyplot as plt

if __name__=="__main__":

    # Load Jackson's distribution
    with open("test/test_circ_dist.txt", 'r') as jackson_file:
        gamma_jackson = np.genfromtxt(jackson_file)

    # Parameters
    input_dict = {
        "solver" : {
            "type" : "scipy_fsolve"
        },
        "units" : "English",
        "scene" : {
            "atmosphere" : {},
            "aircraft" : {}
        }
    }
    airplane_dict = {
        "CG" : [0,0,0],
        "weight" : 10.0,
        "airfoils" : {
            "NACA_0012" : {
                "CL,a" : 6.907213339669221,
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
    state = {
        "velocity" : 1.0,
        "alpha" : 4.2
    }

    # Initialize
    scene = MUX.Scene(input_dict)
    scene.add_aircraft("jackson_wing", airplane_dict, state=state)

    scene.display_wireframe(show_vortices=True)

    scene._solve_w_scipy(verbose=True)

    plt.figure()
    plt.plot(gamma_jackson[:,0], gamma_jackson[:,1], label='Jackson')
    plt.plot(scene._PC[:,1], scene._gamma, label='MUX')
    plt.legend()
    plt.xlabel("Span Location")
    plt.ylabel("Circulation")
    plt.show()