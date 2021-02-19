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
            "NACA_0010" : {
                "geometry" : {
                    "NACA" : "0010"
                }
            }
        },
        "plot_lacs" : False,
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "airfoil" : [[0.0, "NACA_0010"],
                             [1.0, "NACA_0010"]],
                "semispan" : 6.0,
                "dihedral" : [[0.0, 0.0],
                              [1.0, 0.0]],
                "chord" : [[0.0, 1.0],
                           [1.0, 1.0]],
                "sweep" : [[0.0, 45.0],
                           [1.0, 45.0]],
                "grid" : {
                    "N" : 55
                },
                "CAD_options" :{
                    "round_wing_tip" : True,
                    "round_wing_root" : False,
                    "n_rounding_sections" : 30
                }
            }
        }
    }

    # Specify state
    state = {
        "position" : [0.0, 0.0, 0.0],
        "velocity" : 100.0,
        "alpha" : 5.0,
        "beta" : 0.0,
        "orientation" : [0.0, 0.0, 0.0]
    }

    # Load scene
    scene = MX.Scene(input_dict)
    scene.add_aircraft("plane", airplane_dict, state=state)

    # Export vtk
    vtk_file = "swept_wing.vtk"
    scene.export_vtk(filename="swept_wing.vtk", section_resolution=61)

    my_mesh = pp.Mesh(mesh_file=vtk_file, mesh_file_type="VTK", kutta_angle=90.0, verbose=True)
    my_mesh.export_vtk("swept_wing_pp.vtk")
    solver = pp.VortexRingSolver(mesh=my_mesh, verbose=True)
    solver.set_condition(V_inf=[-100.0, 0.0, -10.0], rho=0.0023769)
    FM = solver.solve(lifting=True, verbose=True)
    print(FM)
    solver.export_vtk("case.vtk")
