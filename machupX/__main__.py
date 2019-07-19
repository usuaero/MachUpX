"""A wrapper to the Scene class that lets MachUpX behave like MachUp Pro.
   Takes the input JSON as an argument and runs all analyses specified by 
   the "run" key."""

from .scene import Scene
import sys
import json

# Get input filename from command line arguments
input_filename = sys.argv[-1]

# Initialize scene
scene = Scene(input_filename)

# Import JSON to determine which analyses the user wants to run
with open(input_filename) as json_file_handle:
    input_dict = json.load(json_file_handle)

# Run analyses
for key in input_dict["run"]:
    print(key)