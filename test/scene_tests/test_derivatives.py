# Tests the functionality of the derivative special functions

import machupX as MX
import numpy as np
import json

input_file = "test/input_for_testing.json"

def test_stability_derivatives():
    # Tests the calculation of the stability derivatives

    # Load scene
    scene = MX.Scene(input_file)
    stab_derivs = scene.stability_derivatives()

    assert abs(stab_derivs["test_plane"]["CL,a"]-6.322012906729068)<1e-10
    assert abs(stab_derivs["test_plane"]["CD,a"]-0.3246111615552763)<1e-10
    assert abs(stab_derivs["test_plane"]["CS,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cx,a"]-0.11707169911153548)<1e-10
    assert abs(stab_derivs["test_plane"]["Cy,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cz,a"]+6.336555845802937)<1e-10
    assert abs(stab_derivs["test_plane"]["Cl,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cm,a"]+3.946036942099846)<1e-10
    assert abs(stab_derivs["test_plane"]["Cn,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["CL,b"])<1e-10
    assert abs(stab_derivs["test_plane"]["CD,b"])<1e-10
    assert abs(stab_derivs["test_plane"]["CS,b"]+0.8157254960313394)<1e-9
    assert abs(stab_derivs["test_plane"]["Cx,b"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cy,b"]+0.8304002728742348)<1e-9
    assert abs(stab_derivs["test_plane"]["Cz,b"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cl,b"]+0.1615386223535695)<1e-10
    assert abs(stab_derivs["test_plane"]["Cm,b"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cn,b"]-0.6171846005680138)<1e-9


def test_damping_derivatives():
    # Test the calculation of the damping derivatives

    # Load scene
    scene = MX.Scene(input_file)
    damp_derivs = scene.damping_derivatives()

    assert abs(damp_derivs["test_plane"]["CL,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["CD,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["CS,pbar"]+0.07876155824189048)<1e-9
    assert abs(damp_derivs["test_plane"]["Cx,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cy,pbar"]+0.07876155824189048)<1e-9
    assert abs(damp_derivs["test_plane"]["Cz,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cl,pbar"]+2.5582645049471457)<1e-10
    assert abs(damp_derivs["test_plane"]["Cm,pbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cn,pbar"]-0.014280792399212826)<1e-9
    assert abs(damp_derivs["test_plane"]["CL,qbar"]-12.953376416553475)<1e-9
    assert abs(damp_derivs["test_plane"]["CD,qbar"]-0.30679384901623197)<1e-10
    assert abs(damp_derivs["test_plane"]["CS,qbar"])<1e-9
    assert abs(damp_derivs["test_plane"]["Cx,qbar"]-0.14545935904301463)<1e-10
    assert abs(damp_derivs["test_plane"]["Cy,qbar"])<1e-9
    assert abs(damp_derivs["test_plane"]["Cz,qbar"]+12.956192520550136)<1e-9
    assert abs(damp_derivs["test_plane"]["Cl,qbar"])<1e-10
    assert abs(damp_derivs["test_plane"]["Cm,qbar"]+36.46247702820127)<1e-9
    assert abs(damp_derivs["test_plane"]["Cn,qbar"])<1e-9
    assert abs(damp_derivs["test_plane"]["CL,rbar"])<1e-9
    assert abs(damp_derivs["test_plane"]["CD,rbar"])<1e-9
    assert abs(damp_derivs["test_plane"]["CS,rbar"]-1.225447113641625)<1e-9
    assert abs(damp_derivs["test_plane"]["Cx,rbar"])<1e-9
    assert abs(damp_derivs["test_plane"]["Cy,rbar"]-1.2254471136479639)<1e-8
    assert abs(damp_derivs["test_plane"]["Cz,rbar"])<1e-9
    assert abs(damp_derivs["test_plane"]["Cl,rbar"]-0.3681357898633655)<1e-9
    assert abs(damp_derivs["test_plane"]["Cm,rbar"])<1e-9
    assert abs(damp_derivs["test_plane"]["Cn,rbar"]+0.9272539506646101)<1e-8


def test_control_derivatives():
    # Test the calculation of the control derivatives

    # Load scene
    scene = MX.Scene(input_file)
    cont_derivs = scene.control_derivatives()
    print(json.dumps(cont_derivs, indent=4))

    assert abs(cont_derivs["test_plane"]["CL,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["CD,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["CS,daileron"]-0.13133577886049178)<1e-10
    assert abs(cont_derivs["test_plane"]["Cx,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cy,daileron"]-0.13133577886049178)<1e-10
    assert abs(cont_derivs["test_plane"]["Cz,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cl,daileron"]+0.4830877921597109)<1e-10
    assert abs(cont_derivs["test_plane"]["Cm,daileron"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cn,daileron"]+0.11524853834347693)<1e-10
    assert abs(cont_derivs["test_plane"]["CL,delevator"]-1.6788585952520692)<1e-10
    assert abs(cont_derivs["test_plane"]["CD,delevator"]-0.030922728990260714)<1e-10
    assert abs(cont_derivs["test_plane"]["CS,delevator"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cx,delevator"]-0.027687428309700814)<1e-10
    assert abs(cont_derivs["test_plane"]["Cy,delevator"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cz,delevator"]+1.678915067635511)<1e-10
    assert abs(cont_derivs["test_plane"]["Cl,delevator"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cm,delevator"]+4.973554394205788)<1e-10
    assert abs(cont_derivs["test_plane"]["Cn,delevator"])<1e-10
    assert abs(cont_derivs["test_plane"]["CL,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["CD,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["CS,drudder"]+0.6389847372823805)<1e-10
    assert abs(cont_derivs["test_plane"]["Cx,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cy,drudder"]+0.6389847372823805)<1e-10
    assert abs(cont_derivs["test_plane"]["Cz,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cl,drudder"]+0.13927877375587006)<1e-10
    assert abs(cont_derivs["test_plane"]["Cm,drudder"])<1e-10
    assert abs(cont_derivs["test_plane"]["Cn,drudder"]-0.511515145687878)<1e-10


def test_all_derivs():
    # Test the calculation of the derivatives

    # Load scene
    scene = MX.Scene(input_file)
    derivs = scene.derivatives()

    assert abs(derivs["test_plane"]["stability"]["CL,a"]-6.322012906729068)<1e-10
    assert abs(derivs["test_plane"]["stability"]["CD,a"]-0.3246111615552763)<1e-10
    assert abs(derivs["test_plane"]["stability"]["CS,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cx,a"]-0.11707169911153548)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cy,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cz,a"]+6.336555845802937)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cl,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cm,a"]+3.946036942099846)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cn,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CL,b"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CD,b"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CS,b"]+0.8157254960313394)<1e-9
    assert abs(derivs["test_plane"]["stability"]["Cx,b"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cy,b"]+0.8304002728742348)<1e-9
    assert abs(derivs["test_plane"]["stability"]["Cz,b"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cl,b"]+0.1615386223535695)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cm,b"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cn,b"]-0.6171846005680138)<1e-9
    assert abs(derivs["test_plane"]["damping"]["CL,pbar"])<1e-10
    assert abs(derivs["test_plane"]["damping"]["CD,pbar"])<1e-10
    assert abs(derivs["test_plane"]["damping"]["CS,pbar"]+0.07876155824189048)<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cx,pbar"])<1e-10
    assert abs(derivs["test_plane"]["damping"]["Cy,pbar"]+0.07876155824189048)<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cz,pbar"])<1e-10
    assert abs(derivs["test_plane"]["damping"]["Cl,pbar"]+2.5582645049471457)<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cm,pbar"])<1e-10
    assert abs(derivs["test_plane"]["damping"]["Cn,pbar"]-0.014280792399212826)<1e-9
    assert abs(derivs["test_plane"]["damping"]["CL,qbar"]-12.953376416553475)<1e-9
    assert abs(derivs["test_plane"]["damping"]["CD,qbar"]-0.30679384901623197)<1e-9
    assert abs(derivs["test_plane"]["damping"]["CS,qbar"])<1e-8
    assert abs(derivs["test_plane"]["damping"]["Cx,qbar"]-0.14545935904301463)<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cy,qbar"])<1e-8
    assert abs(derivs["test_plane"]["damping"]["Cz,qbar"]+12.956192520550136)<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cl,qbar"])<1e-10
    assert abs(derivs["test_plane"]["damping"]["Cm,qbar"]+36.46247702820127)<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cn,qbar"])<1e-8
    assert abs(derivs["test_plane"]["damping"]["CL,rbar"])<1e-9
    assert abs(derivs["test_plane"]["damping"]["CD,rbar"])<1e-9
    assert abs(derivs["test_plane"]["damping"]["CS,rbar"]-1.2254471136479639)<1e-8
    assert abs(derivs["test_plane"]["damping"]["Cx,rbar"])<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cy,rbar"]-1.2254471136479639)<1e-8
    assert abs(derivs["test_plane"]["damping"]["Cz,rbar"])<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cl,rbar"]-0.3681357898633655)<1e-8
    assert abs(derivs["test_plane"]["damping"]["Cm,rbar"])<1e-9
    assert abs(derivs["test_plane"]["damping"]["Cn,rbar"]+0.9272539506646101)<1e-8