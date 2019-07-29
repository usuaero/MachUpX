# Tests that the NLL algorithm is correctly solved

import machupX as MX
import numpy as np
import json
import subprocess as sp

input_file = "test/NLL_tests/input_for_NLL_testing.json"

def test_NLL():
    # Tests the NLL algorithm is correctly solved

    # Create scene
    scene = MX.Scene(input_file)
    FM = scene.solve_forces()
    assert abs(FM["test_plane"]["total"]["Fx"]+0.3218660582470263)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+16.372019206586774)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"])<1e-10
    assert abs(FM["test_plane"]["total"]["My"])<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"])<1e-10