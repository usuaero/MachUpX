# This script is for me to test the functionality of whatever I'm working on at the moment.
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
            "type" : "nonlinear"
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
        "controls" : {
            "aileron" : {
                "is_symmetric" : False
            },
            "elevator" : {
                "is_symmetric" : True
            },
            "rudder" : {
                "is_symmetric" : False
            }
        },
        "airfoils" : {
            "NACA_4410" : "dev/NACA_4410.json",
            #"NACA_0010" : {
            #    "geometry" : {
            #        "outline_points" : "dev/Eppler_335.txt"
            #    },
            #    "camber_solver_kwargs" : {
            #        "camber_termination_tol" : 1e-8,
            #        "verbose" : True
            #    }
            #}
            "NACA_0010" : "dev/NACA_0010.json"
        },
        "plot_lacs" : False,
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "airfoil" : [[0.0, "NACA_0010"],
                             [0.3, "NACA_0010"],
                             [0.7, "NACA_0010"],
                             [1.0, "NACA_0010"]],
                "semispan" : 4.0,
                "dihedral" : [[0.0, 0.0],
                              [0.5, 0.0],
                              [0.5, 0.0],
                              [1.0, 0.0]],
                "twist" : [[0.0, 0.0],
                           [0.5, 0.0],
                           [0.5, 0.0],
                           [1.0, 0.0]],
                "sweep" : [[0.0, 45.0],
                           [0.5, 45.0],
                           [0.5, 45.0],
                           [1.0, 45.0]],
                #"control_surface" : {
                #    "chord_fraction" : 0.4,
                #    "root_span" : 0.55,
                #    "tip_span" : 0.95,
                #    "control_mixing" : {
                #        "aileron" : 1.0,
                #        "elevator" : 1.0
                #    }
                #},
                "grid" : {
                    "N" : 40,
                    "wing_ID" : 1,
                    "reid_corrections" : True
                    #"joint_length" : 2.0,
                    #"blending_distance" : 2.0
                },
                "CAD_options" :{
                    "round_stl_tip" : True,
                    "round_stl_root" : False,
                    "n_rounding_sections" : 10
                }
            }
        }
    }

    # Specify state
    state = {
        "velocity" : 100.0,
        "alpha" : 0.0,
        "beta" : 0.0
    }

    control_state = {
        "elevator" : 0.0,
        "aileron" : 0.0,
        "rudder" : 0.0
    }

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)

    #scene.display_wireframe(show_vortices=True)
    stl_file = "swept_wing_40_span_41_sec_10_tip.stl"
    scene.export_stl(filename=stl_file, section_resolution=41)

    ## Solve forces
    #FM = scene.solve_forces(non_dimensional=False, verbose=True)
    #print(json.dumps(FM["plane"]["total"], indent=4))
    #scene.out_gamma()

    #scene.distributions(filename="dist.txt")

    ## Get derivatives
    #derivs = scene.derivatives(wind_frame=False)
    #print(json.dumps(derivs["plane"], indent=4))

    ## Get state derivatives
    #derivs = scene.state_derivatives()
    #print(json.dumps(derivs["plane"], indent=4))

    my_mesh = pp.Mesh(mesh_file=stl_file, mesh_file_type="STL", kutta_angle=90.0, verbose=True)
    #my_mesh.plot(centroids=False)
    solver = pp.VortexRingSolver(mesh=my_mesh, verbose=True)
    solver.set_condition(V_inf=[-100.0, 0.0, -10.0], rho=0.0023769)
    FM = solver.solve(lifting=True, verbose=True)
    print(FM)
    solver.export_vtk("case.vtk")
