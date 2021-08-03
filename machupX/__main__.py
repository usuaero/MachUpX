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
    for key, params in input_dict["run"].items():

        # Specify filename
        if key == "export_stl":
            filename = params.pop("filename", input_filename.replace(".json", ".stl"))
        elif key == "export_vtk":
            filename = params.pop("filename", input_filename.replace(".json", ".vtk"))
        elif key == "distributions":
            filename = params.pop("filename", input_filename.replace(".json", "_distributions.txt"))
        elif "display" in key:
            filename = params.pop("filename", None)
        else:
            filename = params.pop("filename", input_filename.replace(".json", "_"+key+".json"))

        # Call
        try:
            print()
            print("Calling method {0}...".format(key), end='')
            getattr(scene, key)(filename=filename, **params)
            print("Done")
        
        except AttributeError:
            print("{0} is not recognized as a valid run command. Skipping...".format(key))

    print("\nCompleted prescribed analyses.")


if __name__=="__main__":
    
    # Print sweet logo
    print()
    print('--------------------------------------------------------------------------------------------')
    print('|        M    A     C     H     U     P                                                    |')
    print('|      _____________       _____________                                                   |')
    print('|      \            \     /            /                                                   |')
    print('|       \            \   /            /                    MachUpX 2.7.0                   |')
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
    print('--------------------------------------------------------------------------------------------')
    print('|                              MachUpX exited successfully.                                |')
    print('|                                      Thank you!                                          |')
    print('--------------------------------------------------------------------------------------------')
