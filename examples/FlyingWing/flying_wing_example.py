"""
The purpose of this script is to demonstrate the functionality of MachUpX for modelling
a flying wing. Run this script using:

$ python flying_wing_example.py

The input file is also written such that the same analyses will be performed if run using
the `python -m` command. This is done using:

$ python -m machupX flying_wing_input.json
"""

# Import the MachUpX module
import machupX as MX

# Input the json module to make displaying dictionaries more friendly
import json

if __name__=="__main__":

    # Define the input file. The input file will contain the path to the aircraft
    # file, and so this does not need to be defined here.
    input_file = "flying_wing_input.json"

    # Initialize Scene object. This contains the airplane and all necessary
    # atmospheric data.
    my_scene = MX.Scene(input_file)

    # We are now ready to perform analyses. display_wireframe will let us look at 
    # the aircraft we have created. To make sure we know where each lifting surface 
    # is, we'll set show_legend to true.
    my_scene.display_wireframe(show_legend=True)

    # Let's see what forces are acting on the airplane. We'll output just the total
    # dimensional forces and moments acting on the airplane. Note we need to know 
    # the name of the airplane to be able to access its data.
    FM_results = my_scene.solve_forces(dimensional=True, non_dimensional=False, verbose=True)
    print(json.dumps(FM_results["flying_wing"]["total"], indent=4))

    # Now let's get the airplane to its trim state in pitch. MachUpX will default to 
    # Using the 'elevator' control to trim out the airplane. We can use set_trim_state 
    # to have MachUpX set the trim state to be the new state of the airplane.
    trim_state = my_scene.aircraft_pitch_trim(set_trim_state=True, verbose=True)
    print(json.dumps(trim_state["flying_wing"], indent=4))

    # Now that we're trimmed, let's see what our aerodynamic derivatives are.
    derivs = my_scene.aircraft_derivatives()
    print(json.dumps(derivs["flying_wing"], indent=4))
