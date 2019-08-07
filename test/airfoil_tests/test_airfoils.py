# Tests the functionality of airfoil getters

import machupX as MX
import numpy as np
import json

airfoil_file = "test/airfoils_for_testing.json"


def test_default_airfoil_get_CLa():
    # Tests the default airfoil has a lift slope of 2pi
    airfoil = MX.airfoil.Airfoil("my_airfoil")
    params = np.random.rand(5)
    CLa = airfoil.get_CLa(params)
    assert CLa == 2*np.pi


def test_airfoil_get_CLa():
    # Tests the airoil CLa getter
    with open(airfoil_file, 'r') as airfoil_db_handle:
        airfoil_db = json.load(airfoil_db_handle)

    airfoil = MX.airfoil.Airfoil("NACA_0010", airfoil_db["NACA_0010"])
    params = np.random.rand(5)
    CLa = airfoil.get_CLa(params)
    assert CLa == 6.4336


def test_airfoil_get_aL0():
    # Tests the airoil aL0 getter
    with open(airfoil_file, 'r') as airfoil_db_handle:
        airfoil_db = json.load(airfoil_db_handle)

    airfoil = MX.airfoil.Airfoil("NACA_2410", airfoil_db["NACA_2410"])
    params = np.random.rand(5)
    aL0 = airfoil.get_aL0(params)
    assert aL0 == -0.0368