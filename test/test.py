# This script is for me to test the functionality of whatever I'm working on at the moment.
import machupX as MX
import json
import numpy as np
import subprocess as sp
import matplotlib.pyplot as plt
from stl import mesh
from mpl_toolkits import mplot3d

if __name__=="__main__":
    
    input_file = "test/input_for_testing.json"

    # Alter input
    with open(input_file, 'r') as input_handle:
        input_dict = json.load(input_handle)

    with open(input_dict["scene"]["aircraft"]["test_plane"]["file"], 'r') as airplane_file_handle:
        airplane_dict = json.load(airplane_file_handle)

    airplane_state = input_dict["scene"]["aircraft"].pop("test_plane")
    state = airplane_state.get("state", {})
    control_state = airplane_state.get("control_state", {})

    # Load scene
    scene = MX.Scene(input_dict)

    airplane_dict["wings"]["main_wing"]["airfoil"] = "NACA_2410"
    airplane_dict["wings"]["main_wing"]["dihedral"] = 20
   
    scene.add_aircraft("test_plane", airplane_dict, state=state, control_state=control_state)

    # Get outline vectors
    vectors = scene._airplanes["test_plane"].wing_segments["main_wing_left"].get_stl_vectors()

    # Create stl mesh
    segment = mesh.Mesh(np.zeros(vectors.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(vectors):
        for j in range(3):
            segment.vectors[i][j] = f[j]

    # plot
    fig = plt.figure()
    ax = mplot3d.Axes3D(fig)
    ax.add_collection3d(mplot3d.art3d.Poly3DCollection(segment.vectors))
    scale = segment.points.flatten(-1)
    ax.auto_scale_xyz(scale, scale, scale)
    plt.show()

    segment.save("wing.stl")