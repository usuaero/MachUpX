"""A wrapper to the Scene class that lets MachUpX behave like MachUp Pro.
   Takes the input JSON as an argument and runs all analyses specified by 
   the "run" key."""

from .scene import Scene
import sys
import json

# Get input filename from command line arguments
input_filename = sys.argv[-1]

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

# Initialize scene
scene = Scene(input_filename)

# Import JSON to determine which analyses the user wants to run
with open(input_filename) as json_file_handle:
    input_dict = json.load(json_file_handle)

# Run analyses
for key in input_dict["run"]:
    params = input_dict["run"].get(key, {})
    if key == "forces":
        filename = params.get("output_file", input_filename.replace(".json", "_forces.json"))
        non_dimensional = params.get("non_dimensional", False)
        verbose = params.get("verbose", False)

        print("\nCalculating aerodynamic forces...")
        scene.solve_forces(filename=filename, non_dimensional=non_dimensional, verbose=verbose)

    elif key == "view":
        show_legend = params.get("show_legend", False)

        print("Displaying wireframe...")
        scene.display_wireframe(show_legend=show_legend)