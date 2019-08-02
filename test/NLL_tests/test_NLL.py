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
    assert abs(FM["test_plane"]["total"]["FL"]-20.96342459325974)<1e-10
    assert abs(FM["test_plane"]["total"]["FD"]-1.3936939776023654)<1e-10
    assert abs(FM["test_plane"]["total"]["FS"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fx"]+0.6612320094219648)<1e-10
    assert abs(FM["test_plane"]["total"]["Fy"])<1e-10
    assert abs(FM["test_plane"]["total"]["Fz"]+20.999293459785942)<1e-10
    assert abs(FM["test_plane"]["total"]["Mx"])<1e-10
    assert abs(FM["test_plane"]["total"]["My"]+12.88084567119999)<1e-10
    assert abs(FM["test_plane"]["total"]["Mz"])<1e-10