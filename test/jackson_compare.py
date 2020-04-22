import machupX as MUX
import numpy as np
import matplotlib.pyplot as plt
import json

np.set_printoptions(linewidth=np.inf)

if __name__=="__main__":

    # Load Jackson's distribution
    with open("test/test_circ_dist.txt", 'r') as jackson_file:
        gamma_jackson = np.genfromtxt(jackson_file)

    with open("test/velocities.dat", 'r') as jackson_file:
        vel_jackson = np.genfromtxt(jackson_file)

    with open("test/aero_props.dat", 'r') as jackson_file:
        section_jackson = np.genfromtxt(jackson_file)

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
                    "N" : 320,
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

    not_grid = 1
    if not_grid:

        # Initialize
        scene = MUX.Scene(input_dict)
        scene.add_aircraft("jackson_wing", airplane_dict, state=state)

        #scene.display_wireframe(show_vortices=True)

        FM = scene.solve_forces(verbose=True, report_by_segment=True, non_dimensional=False)
        print(json.dumps(FM["jackson_wing"]["total"], indent=4))

        #plt.figure()
        #plt.title("x Velocity")
        #plt.plot(scene._PC[:,1], -scene._v_i[:,0], label='MUX')
        #plt.plot(vel_jackson[:,0], vel_jackson[:,1], label='Jackson')
        #plt.legend()
        #plt.show()

        #plt.figure()
        #plt.title("y Velocity")
        #plt.plot(scene._PC[:,1], scene._v_i[:,1], label='MUX')
        #plt.plot(vel_jackson[:,0], vel_jackson[:,3], label='Jackson')
        #plt.legend()
        #plt.show()

        #plt.figure()
        #plt.title("z Velocity")
        #plt.plot(scene._PC[:,1], -scene._v_i[:,2], label='MUX')
        #plt.plot(vel_jackson[:,0], vel_jackson[:,2], label='Jackson')
        #plt.legend()
        #plt.show()

        plt.figure()
        plt.plot(gamma_jackson[:,0], gamma_jackson[:,1], label='Jackson')
        plt.plot(scene._PC[:,1], scene._gamma, label='MUX')
        plt.legend()
        plt.xlabel("Span Location")
        plt.ylabel("Circulation")
        plt.show()

    else:

        grids = [10, 20, 40, 80, 160, 320, 640]

        plt.figure()
        plt.plot(gamma_jackson[:,0], gamma_jackson[:,1], label='Jackson')
        for grid in grids:
            airplane_dict["wings"]["main_wing"]["grid"]["N"] = grid
            scene = MUX.Scene(input_dict)
            scene.add_aircraft("jackson_wing", airplane_dict, state=state)
            scene._solve_w_scipy(verbose=True)
            plt.plot(scene._PC[:,1], scene._gamma, label='MUX {0}'.format(grid))
        plt.legend()
        plt.xlabel("Span Location")
        plt.ylabel("Circulation")
        plt.show()