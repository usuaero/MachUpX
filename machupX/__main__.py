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

        print('Sorry, interactive mode is not yet functional. Try back later.')
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
        if key == "forces":
            filename = params.get("output_file", input_filename.replace(".json", "_forces.json"))
            non_dimensional = params.get("non_dimensional", False)
            verbose = params.get("verbose", False)
    
            print("\nCalculating aerodynamic forces...")
            scene.solve_forces(filename=filename, non_dimensional=non_dimensional, verbose=verbose)
    
        # Wireframe
        elif key == "display_wireframe":
            show_legend = params.get("show_legend", False)
    
            print("\nDisplaying wireframe...")
            scene.display_wireframe(show_legend=show_legend)

        elif key == "aero_derivatives":
            aircraft_name = params.get("aircraft_name", None)
            filename = params.get("output_file", input_filename.replace(".json", "_derivatives.json"))

            print("\nCalculating aerodynamic derivatives...")
            scene.aircraft_derivatives(aircraft_name=aircraft_name, filename=filename)


if __name__=="__main__":
    
    # Print sweet logo
    print()
    print('-----------------------------------------------')
    print('|                                             |')
    print('|                         BBBBB               |')
    print('|                       BB   BB               |')
    print('|                     BB     BB               |')
    print('|                    BB      BB               |')
    print('|                  BB        BB               |')
    print('|                 BB         BB               |')
    print('|               BB           BB               |')
    print('|              BB        BBBBBB               |')
    print('|                                             |')
    print('|                MachUpX 0.1                  |')
    print('|                                             |')
    print('|        (c) USU Aero Lab, LLC, 2019          |')
    print('|                                             |')
    print('|          This software comes with           |')
    print('| ABSOLUTELY NO WARRANTY EXPRESSED OR IMPLIED |')
    print('|                                             |')
    print('|           Submit bug reports to:            |')
    print('|           doug.hunsaker@usu.edu             |')
    print('-----------------------------------------------')

    # Check for interactive mode
    if "-i" in sys.argv:
        _run_interactive_mode()
    else:
        # Get input filename from command line arguments
        input_filename = sys.argv[-1]
        _run_prescribed_analyses(input_filename)
