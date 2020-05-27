import numpy as np
import matplotlib.pyplot as plt
import subprocess as sp
import machupX as mx
import json
import copy


def translate_to_machup_pro(machupx_input, machupx_airplane, state, control_state):
    """Takes the MachUpX input and airplane and runs them in MachUp Pro for comparison.
    """

    # Initialize
    mu_pro_dict = {
        "run" : {},
        "solver" : copy.deepcopy(machupx_input["solver"]),
        "plane" : {
            "name" : "plane",
            "CGx" : machupx_airplane["CG"][0],
            "CGy" : machupx_airplane["CG"][1],
            "CGz" : machupx_airplane["CG"][2]
        },
        "reference" : copy.deepcopy(machupx_airplane["reference"]),
        "condition" : copy.deepcopy(state),
        "controls" : copy.deepcopy(machupx_airplane["controls"]),
        "airfoil_DB" : "test/MachUp_Pro_airfoils",
        "wings" : {}
    }

    # Run commands
    for key, value in machupx_input["run"].items():
        if key == "solve_forces":
            mu_pro_dict["run"]["forces"] = ""
        elif key == "aero_derivatives":
            mu_pro_dict["run"]["derivatives"] = ""
        elif key == "export_stl":
            mu_pro_dict["run"]["stl"] = ""

    # Condition
    mu_pro_dict["condition"]["units"] = machupx_input["units"]
    mu_pro_dict["condition"]["density"] = machupx_input["scene"]["atmosphere"]["density"]
    mu_pro_dict["condition"]["W"] = machupx_airplane["weight"]
    mu_pro_dict["condition"]["beta"] = np.degrees(np.arctan2(np.tan(np.radians(mu_pro_dict["condition"].pop("beta", 0.0))), np.cos(np.radians(mu_pro_dict["condition"].get("alpha", 0.0)))))
    w = mu_pro_dict["condition"].pop("angular_rates", [0.0, 0.0, 0.0])
    mu_pro_dict["condition"]["omega"] = {}
    mu_pro_dict["condition"]["omega"]["roll"] = w[0]/mu_pro_dict["condition"]["velocity"]
    mu_pro_dict["condition"]["omega"]["pitch"] = w[1]/mu_pro_dict["condition"]["velocity"]
    mu_pro_dict["condition"]["omega"]["yaw"] = w[2]/mu_pro_dict["condition"]["velocity"]

    # Controls
    for key, value in mu_pro_dict["controls"].items():
        value["deflection"] = control_state.get(key, 0.0)

    # Wings
    for key, value in machupx_airplane["wings"].items():
        mu_pro_dict["wings"][key] = copy.deepcopy(value)
        mu_pro_dict["wings"][key]["name"] = key
        mu_pro_dict["wings"][key]["connect"] = mu_pro_dict["wings"][key].pop("connect_to", {})
        mu_pro_dict["wings"][key]["span"] = mu_pro_dict["wings"][key].pop("semispan")
        mu_pro_dict["wings"][key]["mounting_angle"] = value.get("twist", 0.0)
        mu_pro_dict["wings"][key]["sweep"] = value.get("sweep", 0.0)
        mu_pro_dict["wings"][key]["dihedral"] = value.get("dihedral", 0.0)
        mu_pro_dict["wings"][key]["washout"] = 0.0
        mu_pro_dict["wings"][key]["root_chord"] = value.get("chord", 1.0)
        mu_pro_dict["wings"][key]["tip_chord"] = value.get("chord", 1.0)
        mu_pro_dict["wings"][key]["airfoils"] = {
            value["airfoil"] : "",
            value["airfoil"] : ""
        }
        mu_pro_dict["wings"][key]["grid"] = mu_pro_dict["wings"][key].pop("grid")["N"]
        mu_pro_dict["wings"][key]["control"] = {
            "span_root" : value["control_surface"].get("root_span", 0.0),
            "span_tip" : value["control_surface"].get("tip_span", 1.0),
            "chord_root" : value["control_surface"]["chord_fraction"],
            "chord_tip" : value["control_surface"]["chord_fraction"],
            "mix" : mu_pro_dict["wings"][key]["control_surface"].pop("control_mixing"),
            "is_sealed" : 1
        }
        mu_pro_dict["wings"][key].pop("control_surface", None)

    return mu_pro_dict


def run_machup_pro(input_filename):
    """Interfaces with the Machup executable to run the specified config file."""
    try:
        # Run executable
        print("MachUp is executing {0}...".format(input_filename))
        completed = sp.run(["./test/Machup.out",input_filename])

        # Unexpected completion
        if completed.returncode < 0:
            print("MachUp terminated with code {0} after trying to execute {1}.\n".format(-completed.returncode,input_filename))

        # Successful completion
        else:
            print("MachUp successfully completed executing {0}.\n".format(input_filename))

    # Catch any errors
    except OSError as e:
        raise RuntimeError("MachUp execution failed while executing {0} due to {1}.\n".format(input_filename,e))


if __name__=="__main__":
    
    # Specify input
    input_dict = {
        "run" : {
            "solve_forces" : {},
            "aero_derivatives" : {},
            #"export_stl" : {}
        },
        "solver" : {
            "type" : "nonlinear",
            "correct_sections_for_sweep" : False,
            "machup_pro_deriv" : True,
            "convergence" : 0.0000000001
        },
        "units" : "English",
        "scene" : {
            "atmosphere" : {
                "density" : 0.0023769
            }
        }
    }

    # Specify airplane
    airplane_dict = {
        "CG" : [0.0, 0.0, 0.0],
        "weight" : 50.0,
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
        "reference" : {
            "area" : 8.0,
            "longitudinal_length" : 1.0,
            "lateral_length" : 8.0
        },
        "airfoils" : {
            "NACA_0010" : {
                "type" : "linear",
                "aL0" : 0.0,
                "CLa" : 6.4336,
                "CmL0" : 0.0,
                "Cma" : 0.00,
                "CD0" : 0.00513,
                "CD1" : 0.0,
                "CD2" : 0.0984,
                "geometry" : {
                    "NACA" : "0010"
                }
            }
        },
        "wings" : {
            "main_wing" : {
                "ID" : 1,
                "side" : "both",
                "is_main" : True,
                "connect_to" : {
                    "ID" : 0,
                    "location" : "root",
                    "dx" : 0.0,
                    "dy" : 0.0,
                    "dz" : 0.0,
                    "yoffset" : 0.0
                },
                "semispan" : 4.0,
                "airfoil" : "NACA_0010",
                #"ac_offset" : "kuchemann",
                "dihedral" : 2.0,
                "sweep" : 45.0,
                "control_surface" : {
                    "chord_fraction" : 0.1,
                    "root_span" : 0.5,
                    "tip_span" : 1.0,
                    "control_mixing" : {
                        "aileron" : 1.0
                    }
                },
                "grid" : {
                    "N" : 10,
                    "reid_corrections" : False,
                    "flap_edge_cluster" : False
                }
            },
            "h_stab" : {
                "ID" : 2,
                "side" : "both",
                "is_main" : False,
                "connect_to" : {
                    "ID" : 1,
                    "location" : "root",
                    "dx" : -3.0,
                    "dy" : 0.0,
                    "dz" : 0.0,
                    "yoffset" : 0.0
                },
                "semispan" : 2.0,
                "airfoil" : "NACA_0010",
                "twist" : -2.1,
                #"ac_offset" : "kuchemann",
                "sweep" : 45.0,
                "dihedral" : -10.0,
                "control_surface" : {
                    "chord_fraction" : 0.5,
                    "control_mixing" : {
                        "elevator" : 1.0
                    }
                },
                "grid" : {
                    "N" : 10,
                    "reid_corrections" : False,
                    "flap_edge_cluster" : False
                }
            },
            "v_stab" : {
                "ID" : 3,
                "side" : "right",
                "is_main" : False,
                "connect_to" : {
                    "ID" : 1,
                    "location" : "root",
                    "dx" : -3.0,
                    "dy" : 0.0,
                    "dz" : -0.1,
                    "yoffset" : 0.0
                },
                "semispan" : 2.0,
                "dihedral" : 90.0,
                "airfoil" : "NACA_0010",
                #"ac_offset" : "kuchemann",
                "sweep" : 45.0,
                "control_surface" : {
                    "chord_fraction" : 0.5,
                    "control_mixing" : {
                        "rudder" : 1.0
                    }
                },
                "grid" : {
                    "N" : 10,
                    "reid_corrections" : False,
                    "flap_edge_cluster" : False
                }
            }
        }
    }

    state = {
        "velocity" : 100.0,
        "alpha" : -2.0,
        "beta" : 7.0,
        "angular_rates" : [1.0, 1.0, 1.0]
    }

    control_state = {
        "aileron" : 5.0,
        "elevator" : -3.0,
        "rudder" : 2.0
    }

    # Get MachUp Pro results
    mu_pro_dict = translate_to_machup_pro(input_dict, airplane_dict, state, control_state)
    filename = "pro_input.json"
    with open(filename, 'w') as mu_pro_handle:
        json.dump(mu_pro_dict, mu_pro_handle, indent=4)
    run_machup_pro(filename)
    with open(filename.replace(".json", "_forces.json")) as force_handle:
        FM_pro = json.load(force_handle)

    # Get MachUpX results
    mx_scene = mx.Scene(input_dict)
    mx_scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)
    FM_mx = mx_scene.solve_forces(verbose=True, dimensional=False)
    #mx_scene.export_stl(filename="mux.stl")

    #mx_scene.display_wireframe()

    print("\nMachUp Pro Results")
    print("------------------")
    print(json.dumps(FM_pro["total"]["plane"], indent=4))
    print("\nMachUpX Results")
    print("---------------")
    print(json.dumps(FM_mx["plane"]["total"], indent=4))
    print("\nRatios")
    print("-----")
    print("CL: {0}".format(FM_pro["total"]["plane"]["CL"]/FM_mx["plane"]["total"]["CL"]*int(abs(FM_pro["total"]["plane"]["CL"])>1e-10)))
    print("CD: {0}".format(FM_pro["total"]["plane"]["CD"]/FM_mx["plane"]["total"]["CD"]*int(abs(FM_pro["total"]["plane"]["CD"])>1e-10)))
    print("Cl: {0}".format(FM_pro["total"]["plane"]["Cl"]/FM_mx["plane"]["total"]["Cl"]*int(abs(FM_pro["total"]["plane"]["Cl"])>1e-10)))
    print("Cm: {0}".format(FM_pro["total"]["plane"]["Cm"]/FM_mx["plane"]["total"]["Cm"]*int(abs(FM_pro["total"]["plane"]["Cm"])>1e-10)))
    print("Cn: {0}".format(FM_pro["total"]["plane"]["Cn"]/FM_mx["plane"]["total"]["Cn"]*int(abs(FM_pro["total"]["plane"]["Cn"])>1e-10)))
    print("Cx: {0}".format(FM_pro["total"]["plane"]["CX"]/FM_mx["plane"]["total"]["Cx"]*int(abs(FM_pro["total"]["plane"]["CX"])>1e-10)))
    print("Cy: {0}".format(FM_pro["total"]["plane"]["CY"]/FM_mx["plane"]["total"]["Cy"]*int(abs(FM_pro["total"]["plane"]["CY"])>1e-10)))
    print("Cz: {0}".format(FM_pro["total"]["plane"]["CZ"]/FM_mx["plane"]["total"]["Cz"]*int(abs(FM_pro["total"]["plane"]["CZ"])>1e-10)))
    print("\nErrors")
    print("-----")
    print("CL: {0}%".format(abs((FM_pro["total"]["plane"]["CL"]-FM_mx["plane"]["total"]["CL"])/FM_pro["total"]["plane"]["CL"])*100.0*int(abs(FM_pro["total"]["plane"]["CL"])>1e-10)))
    print("CD: {0}%".format(abs((FM_pro["total"]["plane"]["CD"]-FM_mx["plane"]["total"]["CD"])/FM_pro["total"]["plane"]["CD"])*100.0*int(abs(FM_pro["total"]["plane"]["CD"])>1e-10)))
    print("Cl: {0}%".format(abs((FM_pro["total"]["plane"]["Cl"]-FM_mx["plane"]["total"]["Cl"])/FM_pro["total"]["plane"]["Cl"])*100.0*int(abs(FM_pro["total"]["plane"]["Cl"])>1e-10)))
    print("Cm: {0}%".format(abs((FM_pro["total"]["plane"]["Cm"]-FM_mx["plane"]["total"]["Cm"])/FM_pro["total"]["plane"]["Cm"])*100.0*int(abs(FM_pro["total"]["plane"]["Cm"])>1e-10)))
    print("Cn: {0}%".format(abs((FM_pro["total"]["plane"]["Cn"]-FM_mx["plane"]["total"]["Cn"])/FM_pro["total"]["plane"]["Cn"])*100.0*int(abs(FM_pro["total"]["plane"]["Cn"])>1e-10)))
    print("Cx: {0}%".format(abs((FM_pro["total"]["plane"]["CX"]-FM_mx["plane"]["total"]["Cx"])/FM_pro["total"]["plane"]["CX"])*100.0*int(abs(FM_pro["total"]["plane"]["CX"])>1e-10)))
    print("Cy: {0}%".format(abs((FM_pro["total"]["plane"]["CY"]-FM_mx["plane"]["total"]["Cy"])/FM_pro["total"]["plane"]["CY"])*100.0*int(abs(FM_pro["total"]["plane"]["CY"])>1e-10)))
    print("Cz: {0}%".format(abs((FM_pro["total"]["plane"]["CZ"]-FM_mx["plane"]["total"]["Cz"])/FM_pro["total"]["plane"]["CZ"])*100.0*int(abs(FM_pro["total"]["plane"]["CZ"])>1e-10)))

    #sp.run(['rm', 'pro_input.json'])
    sp.run(['rm', 'pro_input_forces.json'])
    sp.run(['rm', 'pro_input_derivatives.json'])