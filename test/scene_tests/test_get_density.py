# Tests the functionality of the density initialization and getters.

import machupX as MX
import numpy as np
import json
import subprocess as sp

input_file = "test/input_for_testing.json"

def test_unspecified_SI_density():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        density = scene._get_density(position)
        assert np.allclose(density, 1.224999155887, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_unspecified_english_density():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        density = scene._get_density(position)
        assert np.allclose(density, 0.00237689072965, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_specified_english_density_with_SI_units():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"
    input_dict["scene"]["atmosphere"]["rho"] = [0.0023769, "slug/ft^3"]

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        density = scene._get_density(position)
        assert np.allclose(density, 1.225003914881, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_specified_SI_density_with_english_units():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["rho"] = [1.225, "kg/m^3"]

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    for i in range(5): # Test 5 random positions
        position = np.random.random(3)*10000
        density = scene._get_density(position)
        assert np.allclose(density, 0.0023768923675, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_standard_atmos_with_english_units():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["rho"] = "standard"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [0, 5000, 10000, 15000, 20000]
    rhos = [0.0023768907296517925, 0.0020481711609077187, 0.0017555489303339121, 0.0014961556118776782, 0.001267258247483606]

    for alt, rho in zip(alts, rhos): 
        position = np.random.random(3)*10000
        position[2] = -alt
        density = scene._get_density(position)
        assert np.allclose(density, rho, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_standard_atmos_with_SI_units():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"
    input_dict["scene"]["atmosphere"]["rho"] = "standard"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [0, 2000, 4000, 6000, 8000]
    rhos = [1.2249991558877122, 1.0065532169786469, 0.8193463086549038, 0.6601112106182213, 0.5257860233001964]

    for alt, rho in zip(alts, rhos): 
        position = np.random.random(3)*10000
        position[2] = -alt
        density = scene._get_density(position)
        assert np.allclose(density, rho, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_density_array_atmos_profile():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "SI"
    input_dict["scene"]["atmosphere"]["rho"] = [[0.0, 1.0],
                                                [1000.0, 0.5],
                                                [2000.0, 0.4],
                                                [3000.0, 0.2],
                                                [5000.0, 0.18]]

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [500, 1500, 2500, 4000, 4500, 6000]
    rhos = [0.75, 0.45, 0.3, 0.19, 0.185, 0.18]

    for alt, rho in zip(alts, rhos): 
        position = np.random.random(3)*10000
        position[2] = -alt
        density = scene._get_density(position)
        assert np.allclose(density, rho, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_array_atmos_profile_with_unit_conversion():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["rho"] = [[0.0, 1.0],
                                                [1000.0, 0.5],
                                                [2000.0, 0.4],
                                                [3000.0, 0.2],
                                                [5000.0, 0.18],
                                                ["m", "kg/m^3"]]

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [1640.42, 4921.26, 8202.1, 13123.36, 14763.78, 19685.04]
    rhos = [0.0014552402, 0.000873144135, 0.00058209609, 0.000368660857, 0.0003589592555, 0.000349257654]

    for alt, rho in zip(alts, rhos): 
        position = np.random.random(3)*10000
        position[2] = -alt
        density = scene._get_density(position)
        assert np.allclose(density, rho, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_array_atmos_profile_from_file():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["rho"] = "test/scene_tests/density_profile.csv"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [500, 1500, 2500, 3000, 6000, 7000]
    rhos = [0.0019, 0.0017, 0.0015, 0.0014, 0.0008, 0.0008]

    for alt, rho in zip(alts, rhos): 
        position = np.random.random(3)*10000
        position[2] = -alt
        density = scene._get_density(position)
        assert np.allclose(density, rho, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

def test_array_atmos_profile_from_file_with_units():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["rho"] = "test/scene_tests/density_profile_with_units.csv"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    alts = [1640.42, 4921.26, 8202.1, 9842.52, 19685.04, 22965.88]
    rhos = [3.68660857e-6, 3.29854451e-6, 2.91048045e-6, 2.71644842e-6, 1.55225624e-6, 1.55225624e-6]

    for alt, rho in zip(alts, rhos): 
        position = np.random.random(3)*10000
        position[2] = -alt
        density = scene._get_density(position)
        assert np.allclose(density, rho, rtol=0.0, atol=1e-10) == True

    del scene
    sp.run(["rm", altered_input_name])

#TODO: Make the tests for density fields better
def test_atmos_field_from_file():
    # Alter input
    with open(input_file, 'r') as json_handle:
        input_dict = json.load(json_handle)

    input_dict["units"] = "English"
    input_dict["scene"]["atmosphere"]["rho"] = "test/scene_tests/density_field.csv"

    altered_input_name = "input.json"
    with open(altered_input_name, 'w') as json_handle:
        json.dump(input_dict,json_handle)

    scene = MX.Scene(altered_input_name)

    positions = [[500,500,-500],
                 [0,0,-1000],
                 [500,500,0]]

    densities = [0.125,0.15,0.15]

    for position, correct_density in zip(positions,densities):
        density = scene._get_density(position)
        assert np.allclose(density, correct_density, rtol=0, atol=1e-10)

    del scene
    sp.run(["rm", altered_input_name])