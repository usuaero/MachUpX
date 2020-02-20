"""A wrapper to the Scene class that lets MachUpX behave like MachUp Pro.
   Takes the input JSON as an argument and runs all analyses specified by 
   the "run" key."""

from .scene import Scene
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
    scene = Scene(input_filename)
    
    # Import JSON to determine which analyses the user wants to run
    with open(input_filename) as json_file_handle:
        input_dict = json.load(json_file_handle)
    
    # Run analyses
    for key in input_dict["run"]:

        # Get options
        params = input_dict["run"].get(key, {})

        # Solve forces
        if key == "solve_forces":
            filename = params.pop("filename", input_filename.replace(".json", "_forces.json"))
    
            print("\nCalculating aerodynamic forces...")
            scene.solve_forces(filename=filename, **params)
    
        # Wireframe
        elif key == "display_wireframe":
    
            print("\nDisplaying wireframe...")
            scene.display_wireframe(**params)

        # Aerodynamic derivatives
        elif key == "aero_derivatives":
            filename = params.pop("filename", input_filename.replace(".json", "_derivatives.json"))

            print("\nCalculating aerodynamic derivatives...")
            scene.aircraft_derivatives(filename=filename, **params)

        # Distributions
        elif key == "distributions":
            filename = params.pop("filename", input_filename.replace(".json", "_distributions.txt"))

            print("\nExporting distributions...")
            scene.distributions(filename=filename, **params)

        # Pitch trim
        elif key == "pitch_trim":
            filename = params.pop("filename", input_filename.replace(".json", "_pitch_trim.json"))

            print("\nTrimming aircraft in pitch...")
            scene.aircraft_pitch_trim(filename=filename, **params)

        # Aerodynamic center
        elif key == "aero_center":
            filename = params.pop("filename", input_filename.replace(".json", "_aero_center.json"))

            print("\nCalculating location of aerodynamic center...")
            scene.aircraft_aero_center(filename=filename, **params)

        # MAC
        elif key == "MAC":
            filename = params.pop("filename", input_filename.replace(".json", "_MAC.json"))

            print("\nCalculating mean aerodynamic chord...")
            scene.aircraft_mean_aerodynamic_chord(filename=filename, **params)

        # Export .stl
        elif key == "stl":
            filename = params.pop("filename", input_filename.replace(".json", ".stl"))

            print("\nExporting stl...")
            scene.export_stl(filename=filename, **params)

        # Export .stp
        elif key == "stp":

            print("\nExporting stp...")
            scene.export_aircraft_stp(**params)

        # Export dxf
        elif key == "dxf":

            print("\nExporting dxf...")
            scene.export_aircraft_dxf(**params)

        # Unrecognized command
        else:
            print("{0} is not recognized as a valid run command. Skipping...".format(key))

    print("\nCompleted prescribed analyses. Exiting...")


if __name__=="__main__":
    
    # Print sweet logo
    print()
    print('-----------------------------------------------')
    print('|        M    A     C     H     U     P       |')
    print('|      _____________       _____________      |')
    print('|      \            \     /            /      |')
    print('|       \            \   /            /       |')
    print('|        \            \ /            /        |')
    print('|         \            X            /         |')
    print('|          \          / \          /          |')
    print('|           \        |   |        /           |')
    print('|           /       /     \       \           |')
    print('|          /      /  -   -  \      \          |')
    print('|         /     /  -  | |  -  \     \         |')
    print('|        /    /__-    | |    -__\    \        |')
    print('|       /            _| |_            \       |')
    print('|      /____________/     \____________\      |')
    print('|                                             |')
    print('|              MachUpX 2.0.0                  |')
    print('|                                             |')
    print('|        (c) USU Aero Lab, LLC, 2019          |')
    print('|                                             |')
    print('|          This software comes with           |')
    print('| ABSOLUTELY NO WARRANTY EXPRESSED OR IMPLIED |')
    print('|                                             |')
    print('|        Submit bug reports on Github         |')
    print('|                                             |')
    print('-----------------------------------------------')


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
