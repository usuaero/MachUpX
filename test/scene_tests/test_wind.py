# Tests the functionality of the wind initialization and getters.

import machupX as MX
import numpy as np
import json
import subprocess as sp

input_file = "test/input_for_testing.json"

def test_unspecified_wind():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"

    altered_input_name = "input.json"
    sp.run(["rm", altered_input_name])
    with open("input.json", 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        wind = scene._get_wind(position)
        assert np.allclose(wind, [0,0,0], rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_specified_english_wind_with_SI_units():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"
    input_dict["scene"]["atmosphere"]["V_wind"] = [100, 100, 100, "kn"]

    altered_input_name = "input.json"
    with open("input.json", 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        wind = scene._get_wind(position)
        assert np.allclose(wind, [51.4444444, 51.4444444, 51.4444444], rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])