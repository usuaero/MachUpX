# This script is for me to test the functionality of whatever I'm working on at the moment.
import machupX as MX
import json
import numpy as np
import subprocess as sp
import matplotlib.pyplot as plt

if __name__=="__main__":
    
    input_file = "test/input_for_testing.json"

    # Load scene
    scene = MX.Scene(input_file)

    outline = scene._airplanes["test_plane"]._airfoil_database["NACA_2410"].get_outline_points()
    plt.figure()
    plt.plot(outline[:,0], outline[:,1])
    plt.show()