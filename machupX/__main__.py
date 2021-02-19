"""A wrapper to the Scene class that lets MachUpX behave like MachUp Pro.
   Takes the input JSON as an argument and runs all analyses specified by 
   the "run" key."""

from machupX import Scene
import sys
import json


def _run_interactive_mode():
    # Runs interactive mode, allowing the user to select which analyses they want to run on the fly
    print('Welcome to MachUpX Interactive. Type [?] for a list of available commands.')
    
    while True:
        command = input("MX>>")

        print('Sorry, interactive mode is not yet available. Try back later.')
        break
        
        if command == 'q':
            break

def _run_prescribed_analyses(input_filename):
    # Runs the analyses specified in the .json file

    # Initialize scene
    print("\nLoading aircraft and scene...", end='')
    scene = Scene(input_filename)
    print("Done")
    
    # Import JSON to determine which analyses the user wants to run
    with open(input_filename) as json_file_handle:
        input_dict = json.load(json_file_handle)
    
    # Run analyses
    print("\nRunning prescribed analyses")
    print("---------------------------")
    for key in input_dict["run"]:

        # Get options
        params = input_dict["run"].get(key, {})

        # Solve forces
        if key == "solve_forces":
            filename = params.pop("filename", input_filename.replace(".json", "_forces.json"))
    
            print("\nCalculating aerodynamic forces...", end='')
            scene.solve_forces(filename=filename, **params)

            print("Done")
    
        # Wireframe
        elif key == "display_wireframe":
    
            print("\nDisplaying wireframe...", end='')
            scene.display_wireframe(**params)
            print("Done")

        # Aerodynamic derivatives
        elif key == "derivatives":
            filename = params.pop("filename", input_filename.replace(".json", "_derivatives.json"))

            print("\nCalculating aerodynamic derivatives...", end='')
            scene.derivatives(filename=filename, **params)
            print("Done")

        # Distributions
        elif key == "distributions":
            filename = params.pop("filename", input_filename.replace(".json", "_distributions.txt"))

            print("\nExporting distributions...", end='')
            scene.distributions(filename=filename, **params)
            print("Done")

        # Pitch trim
        elif key == "pitch_trim":
            filename = params.pop("filename", input_filename.replace(".json", "_pitch_trim.json"))

            print("\nTrimming aircraft in pitch...", end='')
            scene.pitch_trim(filename=filename, **params)
            print("Done")

        # Target CL
        elif key == "target_CL":
            filename = params.pop("filename", input_filename.replace(".json", "_target_CL.json"))

            print("\nGetting target CL...", end='')
            scene.target_CL(filename=filename, **params)
            print("Done")

        # Aerodynamic center
        elif key == "aero_center":
            filename = params.pop("filename", input_filename.replace(".json", "_aero_center.json"))

            print("\nCalculating location of aerodynamic center...", end='')
            scene.aero_center(filename=filename, **params)
            print("Done")

        # MAC
        elif key == "MAC":
            filename = params.pop("filename", input_filename.replace(".json", "_MAC.json"))

            print("\nCalculating mean aerodynamic chord...", end='')
            scene.MAC(filename=filename, **params)
            print("Done")

        # Export .stl
        elif key == "export_stl":
            filename = params.pop("filename", input_filename.replace(".json", ".stl"))

            print("\nExporting stl...", end='')
            scene.export_stl(filename=filename, **params)
            print("Done")

        # Export .stp
        elif key == "export_stp":

            print("\nExporting stp...", end='')
            scene.export_stp(**params)
            print("Done")

        # Export dxf
        elif key == "export_dxf":

            print("\nExporting dxf...", end='')
            scene.export_dxf(**params)
            print("Done")

        # Export linearized model
        elif key == "export_pylot_model":

            print("\nExporting Pylot model...", end='')
            scene.export_pylot_model(**params)
            print("Done")

        # Set error suppression
        elif key == "set_err_state":

            print("Setting error state...", end='')
            scene.set_err_state(**params)
            print("Done")

        # Unrecognized command
        else:
            print("{0} is not recognized as a valid run command. Skipping...".format(key))

    print("\nCompleted prescribed analyses.")


if __name__=="__main__":
    
    # Print sweet logo
    print()
    print('--------------------------------------------------------------------------------------------')
    print('|        M    A     C     H     U     P                                                    |')
    print('|      _____________       _____________                                                   |')
    print('|      \            \     /            /                                                   |')
    print('|       \            \   /            /                    MachUpX 2.5.1                   |')
    print('|        \            \ /            /                                                     |')
    print('|         \            X            /                  (c) USU Aero Lab, 2020              |')
    print('|          \          / \          /                                                       |')
    print('|           \        |   |        /                   This software comes with             |')
    print('|           /       /     \       \          ABSOLUTELY NO WARRANTY EXPRESSED OR IMPLIED   |')
    print('|          /      /  -   -  \      \                                                       |')
    print('|         /     /  -  | |  -  \     \               Submit bug reports on Github           |')
    print('|        /    /__-    | |    -__\    \             Support forum on Google Groups          |')
    print('|       /            /   \            \                                                    |')
    print('|      /____________/     \____________\                                                   |')
    print('|                                                                                          |')
    print('--------------------------------------------------------------------------------------------')


    # Check for interactive mode
    if "-i" in sys.argv:
        _run_interactive_mode()

    else:
        # Get input filename from command line arguments
        input_filename = sys.argv[-1]

        # Check for valid input
        if ".json" not in input_filename:
            raise IOError("Please specify a .json input file (got {0}).".format(input_filename))

        # Run
        _run_prescribed_analyses(input_filename)


    # Print quit message
    print()
    print("----------------------------------------------------")
    print("|          MachUpX exited successfully.            |")
    print("|                  Thank you!                      |")
    print("----------------------------------------------------")
