# Tests the functionality of the derivative special functions

import machupX as MX
import numpy as np
import json

input_file = "test/input_for_testing.json"

def test_stability_derivatives():
    # Tests the calculation of the stability derivatives

    # Load scene
    scene = MX.Scene(input_file)
    stab_derivs = scene.aircraft_stability_derivatives()

    assert abs(stab_derivs["test_plane"]["CL,a"]-6.322012906729068)<1e-10
    assert abs(stab_derivs["test_plane"]["CD,a"]-0.3246111615552763)<1e-10
    assert abs(stab_derivs["test_plane"]["CS,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cx,a"]-0.11707169911153548)<1e-10
    assert abs(stab_derivs["test_plane"]["Cy,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cz,a"]+6.336555845802937)<1e-10
    assert abs(stab_derivs["test_plane"]["Cl,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cm,a"]+3.946036942099846)<1e-10
    assert abs(stab_derivs["test_plane"]["Cn,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["CL,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["CD,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["CS,B"]+0.8157254960313394)<1e-10
    assert abs(stab_derivs["test_plane"]["Cx,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cy,B"]+0.8304002728742348)<1e-10
    assert abs(stab_derivs["test_plane"]["Cz,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cl,B"]+0.1615386223535695)<1e-10
    assert abs(stab_derivs["test_plane"]["Cm,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cn,B"]-0.6171846005680138)<1e-10


def test_damping_derivatives():
    # Test the calculation of the damping derivatives

    # Load scene
    scene = MX.Scene(input_file)
    damp_derivs = scene.aircraft_damping_derivatives()
    print(json.dumps(damp_derivs["test_plane"], indent=4))

    # There's something about how NLL calculates these cases that introduces a little more numerical error...
    assert abs(damp_derivs["test_plane"]["CL,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["CD,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["CS,pbar"]+0.07876155824189048)<1e-10
    assert abs(damp_derivs["test_plane"]["Cx,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cy,pbar"]+0.07876155824189048)<1e-10
    assert abs(damp_derivs["test_plane"]["Cz,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cl,pbar"]+2.5582645049471457)<1e-10
    assert abs(damp_derivs["test_plane"]["Cm,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cn,pbar"]-0.014280792399212826)<1e-10
    assert abs(damp_derivs["test_plane"]["CL,qbar"]-12.953376416553475)<1e-10
    assert abs(damp_derivs["test_plane"]["CD,qbar"]-0.30679384901623197)<1e-10
    assert abs(damp_derivs["test_plane"]["CS,qbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cx,qbar"]-0.14545935904301463)<1e-10
    assert abs(damp_derivs["test_plane"]["Cy,qbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cz,qbar"]+12.956192520550136)<1e-10
    assert abs(damp_derivs["test_plane"]["Cl,qbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cm,qbar"]+36.46247702820127)<1e-10
    assert abs(damp_derivs["test_plane"]["Cn,qbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["CL,rbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["CD,rbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["CS,rbar"]-1.2254471136479639)<1e-10
    assert abs(damp_derivs["test_plane"]["Cx,rbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cy,rbar"]-1.2254471136479639)<1e-10
    assert abs(damp_derivs["test_plane"]["Cz,rbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cl,rbar"]-0.3681357898633655)<1e-10
    assert abs(damp_derivs["test_plane"]["Cm,rbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cn,rbar"]+0.9272539506646101)<1e-10


def test_control_derivatives():
    # Test the calculation of the control derivatives

    # Load scene
    scene = MX.Scene(input_file)
    cont_derivs = scene.aircraft_control_derivatives()
    print(json.dumps(cont_derivs["test_plane"], indent=4))

    assert abs(cont_derivs["test_plane"]["CL,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["CD,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["CS,daileron"]-0.13409255196609768)<1e-10
    assert abs(cont_derivs["test_plane"]["Cx,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cy,daileron"]-0.13409255196609768)<1e-10
    assert abs(cont_derivs["test_plane"]["Cz,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cl,daileron"]+0.4829213183114422)<1e-10
    assert abs(cont_derivs["test_plane"]["Cm,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cn,daileron"]+0.1173202524037608)<1e-10
    assert abs(cont_derivs["test_plane"]["CL,delevator"]-1.6795004768670678)<1e-10
    assert abs(cont_derivs["test_plane"]["CD,delevator"]-0.030957629339379567)<1e-10
    assert abs(cont_derivs["test_plane"]["CS,delevator"]+2.7500552332993075e-16)<1e-10
    assert abs(cont_derivs["test_plane"]["Cx,delevator"]-0.02767495056623802)<1e-10
    assert abs(cont_derivs["test_plane"]["Cy,delevator"]+2.7500552332993075e-16)<1e-10
    assert abs(cont_derivs["test_plane"]["Cz,delevator"]+1.6795577762381937)<1e-10
    assert abs(cont_derivs["test_plane"]["Cl,delevator"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cm,delevator"]+4.975416426126367)<1e-10
    assert abs(cont_derivs["test_plane"]["Cn,delevator"]-1.5931710494786176e-16)<1e-10
    assert abs(cont_derivs["test_plane"]["CL,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["CD,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["CS,drudder"]+0.6384357546253706)<1e-10
    assert abs(cont_derivs["test_plane"]["Cx,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cy,drudder"]+0.6384357546253706)<1e-10
    assert abs(cont_derivs["test_plane"]["Cz,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cl,drudder"]+0.1411603229331166)<1e-10
    assert abs(cont_derivs["test_plane"]["Cm,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cn,drudder"]-0.5111158114523209)<1e-10


def test_all_derivs():
    # Test the calculation of the derivatives

    # Load scene
    scene = MX.Scene(input_file)
    derivs = scene.aircraft_derivatives()

    assert abs(derivs["test_plane"]["stability"]["CL,a"]-6.322012906729068)<1e-10
    assert abs(derivs["test_plane"]["stability"]["CD,a"]-0.3246111615552763)<1e-10
    assert abs(derivs["test_plane"]["stability"]["CS,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cx,a"]-0.11707169911153548)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cy,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cz,a"]+6.336555845802937)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cl,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cm,a"]+3.946036942099846)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cn,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CL,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CD,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CS,B"]+0.8157254960313394)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cx,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cy,B"]+0.8304002728742348)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cz,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cl,B"]+0.1615386223535695)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cm,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cn,B"]-0.6171846005680138)<1e-10