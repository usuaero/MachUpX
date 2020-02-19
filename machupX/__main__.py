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
            filename = params.get("filename", input_filename.replace(".json", "_distributions.txt"))
            make_plots = params.get("make_plots", [])

            print("\nCalculating distributions...")
            scene.distributions(filename=filename, make_plots=make_plots)

        # Pitch trim
        elif key == "pitch_trim":
            pitch_control = params.get("pitch_control", "elevator")
            set_trim_state = params.get("set_trim_state", True)
            verbose = params.get("verbose", False)
            filename = params.get("filename", input_filename.replace(".json", "_pitch_trim.json"))

            print("Trimming aircraft in pitch...")
            scene.aircraft_pitch_trim(pitch_control=pitch_control, filename=filename, set_trim_state=set_trim_state, verbose=verbose)

        # Aerodynamic center
        elif key == "aero_center":
            aircraft = params.get("aircraft", None)
            filename = params.get("filename", input_filename.replace(".json", "_aero_center.json"))
            verbose = params.get("verbose", False)

            print("Calculating location of aerodynamic center...")
            scene.aircraft_aero_center(aircraft=aircraft, filename=filename, verbose=verbose)

        # MAC
        elif key == "MAC":
            aircraft = params.get("aircraft", None)
            filename = params.get("filename", input_filename.replace(".json", "_MAC.json"))
            verbose = params.get("verbose", False)

            print("Calculating mean aerodynamic chord...")
            scene.aircraft_mean_aerodynamic_chord(aircraft=aircraft, filename=filename, verbose=verbose)

        # Export .stl
        elif key == "stl":
            aircraft = params.get("aircraft", None)
            res = params.get("section_resolution", 200)
            filename = params.get("filename", input_filename.replace(".json", ".stl"))

            print("Exporting stl...")
            scene.export_stl(filename, section_resolution=res, aircraft=aircraft)

        # Export .stp
        elif key == "stp":
            aircraft = params.get("aircraft", None)
            tag = params.get("file_tag", "")
            res = params.get("section_resolution", 200)
            spline = params.get("spline", False)
            sections = params.get("maintain_sections", True)

            print("Exporting stp...")
            scene.export_aircraft_stp(aircraft, file_tag=tag, section_resolution=res, spline=spline, maintain_sections=sections)

        # Export dxf
        elif key == "dxf":

            print("Exporting dxf...")
            scene.export_aircraft_dxf(**params)

        # Unrecognized command
        else:
            print("{0} is not recognized as a valid run command. Skipping...".format(key))

    print("\nCompleted prescribed analyses. Exiting...")


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
    print('|                MachUpX 1.0                  |')
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
