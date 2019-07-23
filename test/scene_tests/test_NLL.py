# Tests that the NLL algorithm is correctly solved

import machupX as MX
import numpy as np
import json
import subprocess as sp

input_file = "test/airplane_tests/input_for_airplane_testing.json"

def test_NLL():
    # Tests the NLL algorithm is correctly solved

    # Create scene
    scene = MX.Scene(input_file)
    scene.solve_forces()