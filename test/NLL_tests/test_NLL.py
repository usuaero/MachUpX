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
    scene.solve_forces()