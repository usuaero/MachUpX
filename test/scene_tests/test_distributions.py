# Tests the functionality of the distributions() function

import json
import machupX as MX
import numpy as np

input_file = "test/input_for_testing.json"

def test_distributions():

    #Load scene
    scene = MX.Scene(input_file)
    dist = scene.distributions()

    assert np.allclose(dist["test_plane"]["main_wing_right"]["chord"], 1.0, rtol=0.0, atol=1e-10)
    assert np.allclose(dist["test_plane"]["main_wing_right"]["sweep"], 0.0, rtol=0.0, atol=1e-10)
    assert np.allclose(dist["test_plane"]["main_wing_left"]["dihedral"], 0.0, rtol=0.0, atol=1e-10)
    assert np.allclose(dist["test_plane"]["v_stab_right"]["dihedral"], 1.5707963267948966, rtol=0.0, atol=1e-10)
    assert np.allclose(dist["test_plane"]["h_stab_left"]["twist"], 0.0, rtol=0.0, atol=1e-10)