#!/usr/bin/env python

"""A wrapper to the Scene class that lets MachUpX behave like MachUp Pro.
   Takes the input JSON as an argument and runs all analyses specified by 
   the "run" key."""

from machupX import Scene

if __name__=="__main__": # Don't run when imported
    
    # Get input filename from command line arguments
    input_filename = sys.argv.index("machupX.py")+1

    # Initialize scene
    scene = Scene(input_filename)

    # Import JSON to determine which analyses the user wants to run
    with open(input_filename) as json_file_handle:
        input_dict = json.load(json_file_handle)

    # Run analyses
    for key in input_dict["run"]:
        print(key)