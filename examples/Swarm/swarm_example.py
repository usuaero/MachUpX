"""
The purpose of this script is to demonstrate the functionality of MachUpX for modelling
a small swarm of drones. Run this script using:

$ python swarm_example.py

The input file is also written such that the same analyses will be performed if run using
the `python -m` command. This is done using:

$ python -m machupX swarm_input.json
"""

# Import the MachUpX module
import machupX as MX

# Input the json module to make displaying dictionaries more friendly
import json

if __name__=="__main__":

    # Define the input file. The input file will contain the path to the aircraft
    # file, and so this does not need to be defined here.
    input_file = "swarm_input.json"

    # Initialize Scene object. This contains the airplane and all necessary
    # atmospheric data.
    my_scene = MX.Scene(input_file)

    # We are now ready to perform analyses. display_wireframe will let us look at 
    # the aircraft we have created. To make sure we know where each lifting surface 
    # is, we'll set show_legend to true.
    my_scene.display_wireframe(show_legend=True, show_vortices=False)

    # Let's see what forces are acting on the airplane. We'll output just the total
    # dimensional forces and moments acting on the airplane. Note we need to know 
    # the name of the airplane to be able to access its data.
    FM_results = my_scene.solve_forces(dimensional=True, non_dimensional=False, verbose=True)
    print("---Drone0---")
    print(json.dumps(FM_results["drone0"]["total"], indent=4))
    print("---Drone1---")
    print(json.dumps(FM_results["drone1"]["total"], indent=4))
    print("---Drone2---")
    print(json.dumps(FM_results["drone2"]["total"], indent=4))
    print("---Drone3---")
    print(json.dumps(FM_results["drone3"]["total"], indent=4))

    # Let's see what our aerodynamic derivatives are.
    derivs = my_scene.aircraft_derivatives()
    print("---Drone0---")
    print(json.dumps(derivs["drone0"], indent=4))
    print("---Drone1---")
    print(json.dumps(derivs["drone1"], indent=4))
    print("---Drone2---")
    print(json.dumps(derivs["drone2"], indent=4))
    print("---Drone3---")
    print(json.dumps(derivs["drone3"], indent=4))
