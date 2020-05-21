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

    # Condition
    mu_pro_dict["condition"]["units"] = machupx_input["units"]
    mu_pro_dict["condition"]["density"] = machupx_input["scene"]["atmosphere"]["density"]
    mu_pro_dict["condition"]["W"] = machupx_airplane["weight"]
    w = state.get("angular_rates", [0.0, 0.0, 0.0])
    mu_pro_dict["condition"]["omega"] = {}
    mu_pro_dict["condition"]["omega"]["roll"] = w[0]
    mu_pro_dict["condition"]["omega"]["roll"] = w[1]
    mu_pro_dict["condition"]["omega"]["roll"] = w[2]

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
            "aero_derivatives" : {}
        },
        "solver" : {
            "type" : "linear",
            "correct_sections_for_sweep" : False,
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
        "airfoils" : "test/airfoils_for_testing.json",
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
                "ac_offset" : "kuchemann",
                "dihedral" : 0.0,
                "sweep" : 0.0,
                "control_surface" : {
                    "chord_fraction" : 0.1,
                    "root_span" : 0.5,
                    "tip_span" : 1.0,
                    "control_mixing" : {
                        "aileron" : 1.0
                    }
                },
                "grid" : {
                    "N" : 40,
                    "reid_corrections" : False
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
                "ac_offset" : "kuchemann",
                "sweep" : 45.0,
                "control_surface" : {
                    "chord_fraction" : 0.5,
                    "control_mixing" : {
                        "elevator" : 1.0
                    }
                },
                "grid" : {
                    "N" : 40,
                    "reid_corrections" : False
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
                "ac_offset" : "kuchemann",
                "sweep" : 45.0,
                "control_surface" : {
                    "chord_fraction" : 0.5,
                    "control_mixing" : {
                        "rudder" : 1.0
                    }
                },
                "grid" : {
                    "N" : 40,
                    "reid_corrections" : False
                }
            }
        }
    }

    state = {
        "velocity" : 100.0,
        "alpha" : 5.0
    }

    control_state = {
        "aileron" : 0.0
    }

    # Get MachUp Pro results
    mu_pro_dict = translate_to_machup_pro(input_dict, airplane_dict, state, control_state)
    filename = "machup_pro_input.json"
    with open(filename, 'w') as mu_pro_handle:
        json.dump(mu_pro_dict, mu_pro_handle)
    run_machup_pro(filename)
    with open(filename.replace(".json", "_forces.json")) as force_handle:
        FM_pro = json.load(force_handle)

    # Get MachUpX results
    mx_scene = mx.Scene(input_dict)
    mx_scene.add_aircraft("plane", airplane_dict, state=state, control_state=control_state)
    FM_mx = mx_scene.solve_forces(verbose=True, dimensional=False)

    print("\nMachUp Pro Resutls")
    print("------------------")
    print(json.dumps(FM_pro["total"]["plane"], indent=4))
    print("\nMachUpX Resutls")
    print("---------------")
    print(json.dumps(FM_mx["plane"]["total"], indent=4))