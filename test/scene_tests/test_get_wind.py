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
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        wind = scene._get_wind(position).flatten()
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
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        wind = scene._get_wind(position).flatten()
        assert np.allclose(wind, [51.4444444, 51.4444444, 51.4444444], rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_specified_SI_wind_with_english_units():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["V_wind"] = [100, 100, 100, "kph"]

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        wind = scene._get_wind(position).flatten()
        assert np.allclose(wind, [91.13444, 91.13444, 91.13444], rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_wind_array_atmos_profile():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"
    input_dict["scene"]["atmosphere"]["V_wind"] = [[0.0, 100.0, 100.0, 100.0],
                                                [1000.0, 80.0, 120.0, 150.0],
                                                [2000.0, 60.0, 140.0, 150.0],
                                                [3000.0, 40.0, 160.0, 150.0],
                                                [5000.0, 20.0, 180.0, 150.0]]

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [500, 1500, 2500, 4000, 4500, 6000]
    winds = [[90,110,125],
             [70,130,150],
             [50,150,150],
             [30,170,150],
             [25,175,150],
             [20,180,150]]

    for alt, correct_wind in zip(alts, winds): 
        position = np.random.random(3)*10000
        position[2] = -alt
        wind = scene._get_wind(position).flatten()
        assert np.allclose(wind, correct_wind, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_wind_array_atmos_profile_with_unit_conversion():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"
    input_dict["scene"]["atmosphere"]["V_wind"] = [[0.0, 100.0, 100.0, 100.0],
                                                [1000.0, 80.0, 120.0, 150.0],
                                                [2000.0, 60.0, 140.0, 150.0],
                                                [3000.0, 40.0, 160.0, 150.0],
                                                [5000.0, 20.0, 180.0, 150.0],
                                                ["ft","mph","mph","mph"]]

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [152.4, 457.2, 762, 1219.2, 1371.6, 1828.8]
    winds = [[40.2336,49.1744,55.88],
             [31.2928,58.1152,67.056],
             [22.352,67.056,67.056],
             [13.4112,75.9968,67.056],
             [11.176,78.232,67.056],
             [8.9408,80.4672,67.056]]

    for alt, correct_wind in zip(alts, winds): 
        position = np.random.random(3)*10000
        position[2] = -alt
        wind = scene._get_wind(position).flatten()
        assert np.allclose(wind, correct_wind, rtol=0.0, atol=1e-10) == True

    del scene

def test_wind_array_atmos_profile_from_file():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["V_wind"] = "test/scene_tests/wind_profile.csv"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [500, 1500, 2500, 4000, 4500, 6000]
    winds = [[90,110,125],
             [70,130,150],
             [50,150,150],
             [30,170,150],
             [25,175,150],
             [20,180,150]]

    for alt, correct_wind in zip(alts, winds): 
        position = np.random.random(3)*10000
        position[2] = -alt
        wind = scene._get_wind(position).flatten()
        assert np.allclose(wind, correct_wind, rtol=0.0, atol=1e-10) == True

def test_wind_array_atmos_profile_from_file_with_units():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"
    input_dict["scene"]["atmosphere"]["V_wind"] = "test/scene_tests/wind_profile_with_units.csv"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [152.4, 457.2, 762, 1219.2, 1371.6, 1828.8]
    winds = [[40.2336,49.1744,55.88],
             [31.2928,58.1152,67.056],
             [22.352,67.056,67.056],
             [13.4112,75.9968,67.056],
             [11.176,78.232,67.056],
             [8.9408,80.4672,67.056]]

    for alt, correct_wind in zip(alts, winds): 
        position = np.random.random(3)*10000
        position[2] = -alt
        wind = scene._get_wind(position).flatten()
        assert np.allclose(wind, correct_wind, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

#TODO: Make the tests for wind fields better
def test_atmos_wind_field_from_file():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["V_wind"] = "test/scene_tests/wind_field.csv"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    positions = [[500,500,-500],
                 [0,0,-1000],
                 [500,500,0]]

    winds = [[100,100,100],
             [100,100,100],
             [100,100,100]]

    for position, correct_wind in zip(positions,winds):
        wind = scene._get_wind(position).flatten()
        assert np.allclose(wind, correct_wind, rtol=0, atol=1e-10)

    del scene
    sp.run(["rm", altered_input_name])