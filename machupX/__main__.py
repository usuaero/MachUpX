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
            filename = params.get("filename", input_filename.replace(".json", "_forces.json"))
            dimensional = params.get("dimensional", True)
            non_dimensional = params.get("non_dimensional", True)
            verbose = params.get("verbose", True)
    
            print("\nCalculating aerodynamic forces...")
            scene.solve_forces(filename=filename, dimensional=dimensional, non_dimensional=non_dimensional, verbose=verbose)
    
        # Wireframe
        elif key == "display_wireframe":
            show_legend = params.get("show_legend", False)
            filename = params.get("filename", None)
    
            print("\nDisplaying wireframe...")
            scene.display_wireframe(show_legend=show_legend, filename=filename)

        # Aerodynamic derivatives
        elif key == "aero_derivatives":
            aircraft = params.get("aircraft", None)
            filename = params.get("filename", input_filename.replace(".json", "_derivatives.json"))

            print("\nCalculating aerodynamic derivatives...")
            scene.aircraft_derivatives(aircraft=aircraft, filename=filename)

        # Distributions
        elif key == "distributions":
            filename = params.get("filename", input_filename.replace(".json", "_distributions.json"))
            make_plots = params.get("make_plots", [])

            print("\nCalculating distributions...")
            scene.distributions(filename=filename, make_plots=make_plots)

        # Pitch trim
        elif key == "pitch_trim":
            aircraft = params.get("aircraft", None)
            pitch_control = params.get("pitch_control", {})
            iterations = params.get("iterations", 1)
            set_trim_state = params.get("set_trim_state", True)
            verbose = params.get("verbose", False)
            filename = params.get("filename", input_filename.replace(".json", "_trim_angles.json"))

            print("Trimming aircraft in pitch...")
            scene.aircraft_pitch_trim(aircraft=aircraft, pitch_control=pitch_control, filename=filename, iterations=iterations, set_trim_state=set_trim_state, verbose=verbose)


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
