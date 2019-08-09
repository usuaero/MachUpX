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

    assert abs(stab_derivs["test_plane"]["CL,a"]-6.325561939795834)<1e-10
    assert abs(stab_derivs["test_plane"]["CD,a"]-0.32484784717545706)<1e-10
    assert abs(stab_derivs["test_plane"]["CS,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cx,a"]-0.11701146202557859)<1e-10
    assert abs(stab_derivs["test_plane"]["Cy,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cz,a"]+6.340112485605566)<1e-10
    assert abs(stab_derivs["test_plane"]["Cl,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cm,a"]+3.951172935738557)<1e-10
    assert abs(stab_derivs["test_plane"]["Cn,a"])<1e-10
    assert abs(stab_derivs["test_plane"]["CL,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["CD,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["CS,B"]+0.8173565005198824)<1e-10
    assert abs(stab_derivs["test_plane"]["Cx,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cy,B"]+0.8320251585209351)<1e-10
    assert abs(stab_derivs["test_plane"]["Cz,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cl,B"]+0.13833770562075048)<1e-10
    assert abs(stab_derivs["test_plane"]["Cm,B"])<1e-10
    assert abs(stab_derivs["test_plane"]["Cn,B"]-0.6187734326780242)<1e-10


def test_damping_derivatives():
    # Test the calculation of the damping derivatives

    # Load scene
    scene = MX.Scene(input_file)
    damp_derivs = scene.aircraft_damping_derivatives()

    # There's something about how NLL calculates these cases that introduces a little more numerical error...
    assert abs(damp_derivs["test_plane"]["CL,pbar"]-0.00037018702894742184)<1e-6
    assert abs(damp_derivs["test_plane"]["CD,pbar"]+6.60883695078468e-05)<1e-6
    assert abs(damp_derivs["test_plane"]["CS,pbar"]+0.09305320429644524)<1e-6
    assert abs(damp_derivs["test_plane"]["Cx,pbar"]-7.896745125857141e-05)<1e-6
    assert abs(damp_derivs["test_plane"]["Cy,pbar"]+0.09305320429644524)<1e-6
    assert abs(damp_derivs["test_plane"]["Cz,pbar"]+0.0003676550701381398)<1e-6
    assert abs(damp_derivs["test_plane"]["Cl,pbar"]+2.55335879459208)<1e-6
    assert abs(damp_derivs["test_plane"]["Cm,pbar"]+0.0011593362887751812)<1e-6
    assert abs(damp_derivs["test_plane"]["Cn,pbar"]-0.02514714847131455)<1e-6
    assert abs(damp_derivs["test_plane"]["CL,qbar"]-13.019918503532345)<1e-6
    assert abs(damp_derivs["test_plane"]["CD,qbar"]-0.3090180115277938)<1e-6
    assert abs(damp_derivs["test_plane"]["CS,qbar"]+0.013965339163743515)<1e-6
    assert abs(damp_derivs["test_plane"]["Cx,qbar"]-0.14555883677632234)<1e-6
    assert abs(damp_derivs["test_plane"]["Cy,qbar"]+0.013965339163743515)<1e-6
    assert abs(damp_derivs["test_plane"]["Cz,qbar"]+13.022771694040092)<1e-6
    assert abs(damp_derivs["test_plane"]["Cl,qbar"]+0.0011967792488973804)<1e-6
    assert abs(damp_derivs["test_plane"]["Cm,qbar"]+36.6535222940767)<1e-6
    assert abs(damp_derivs["test_plane"]["Cn,qbar"]-0.010480645283789216)<1e-6
    assert abs(damp_derivs["test_plane"]["CL,rbar"]+2.211564265053312e-09)<1e-6
    assert abs(damp_derivs["test_plane"]["CD,rbar"]+4.9465639917478654e-11)<1e-6
    assert abs(damp_derivs["test_plane"]["CS,rbar"]-1.2322179506649182)<1e-6
    assert abs(damp_derivs["test_plane"]["Cx,rbar"]+2.776858604169874e-11)<1e-6
    assert abs(damp_derivs["test_plane"]["Cy,rbar"]-1.2322179506649182)<1e-6
    assert abs(damp_derivs["test_plane"]["Cz,rbar"]-2.2119805986875463e-09)<1e-6
    assert abs(damp_derivs["test_plane"]["Cl,rbar"]-0.36543638451918914)<1e-6
    assert abs(damp_derivs["test_plane"]["Cm,rbar"]-6.248890294102694e-09)<1e-6
    assert abs(damp_derivs["test_plane"]["Cn,rbar"]+0.9323904254656564)<1e-6


def test_control_derivatives():
    # Test the calculation of the control derivatives

    # Load scene
    scene = MX.Scene(input_file)
    cont_derivs = scene.aircraft_control_derivatives()

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
    assert abs(derivs["test_plane"]["stability"]["CL,a"]-6.325561939795834)<1e-10
    assert abs(derivs["test_plane"]["stability"]["CD,a"]-0.32484784717545706)<1e-10
    assert abs(derivs["test_plane"]["stability"]["CS,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cx,a"]-0.11701146202557859)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cy,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cz,a"]+6.340112485605566)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cl,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cm,a"]+3.951172935738557)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cn,a"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CL,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CD,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["CS,B"]+0.8173565005198824)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cx,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cy,B"]+0.8320251585209351)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cz,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cl,B"]+0.13833770562075048)<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cm,B"])<1e-10
    assert abs(derivs["test_plane"]["stability"]["Cn,B"]-0.6187734326780242)<1e-10
    assert abs(derivs["test_plane"]["damping"]["CL,pbar"]-0.00037018702894742184)<1e-6
    assert abs(derivs["test_plane"]["damping"]["CD,pbar"]+6.60883695078468e-05)<1e-6
    assert abs(derivs["test_plane"]["damping"]["CS,pbar"]+0.09305320429644524)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cx,pbar"]-7.896745125857141e-05)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cy,pbar"]+0.09305320429644524)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cz,pbar"]+0.0003676550701381398)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cl,pbar"]+2.55335879459208)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cm,pbar"]+0.0011593362887751812)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cn,pbar"]-0.02514714847131455)<1e-6
    assert abs(derivs["test_plane"]["damping"]["CL,qbar"]-13.019918503532345)<1e-6
    assert abs(derivs["test_plane"]["damping"]["CD,qbar"]-0.3090180115277938)<1e-6
    assert abs(derivs["test_plane"]["damping"]["CS,qbar"]+0.013965339163743515)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cx,qbar"]-0.14555883677632234)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cy,qbar"]+0.013965339163743515)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cz,qbar"]+13.022771694040092)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cl,qbar"]+0.0011967792488973804)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cm,qbar"]+36.6535222940767)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cn,qbar"]-0.010480645283789216)<1e-6
    assert abs(derivs["test_plane"]["damping"]["CL,rbar"]+2.211564265053312e-09)<1e-6
    assert abs(derivs["test_plane"]["damping"]["CD,rbar"]+4.9465639917478654e-11)<1e-6
    assert abs(derivs["test_plane"]["damping"]["CS,rbar"]-1.2322179506649182)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cx,rbar"]+2.776858604169874e-11)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cy,rbar"]-1.2322179506649182)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cz,rbar"]-2.2119805986875463e-09)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cl,rbar"]-0.36543638451918914)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cm,rbar"]-6.248890294102694e-09)<1e-6
    assert abs(derivs["test_plane"]["damping"]["Cn,rbar"]+0.9323904254656564)<1e-6
    assert abs(derivs["test_plane"]["control"]["CL,daileron"])<1e-10
    assert abs(derivs["test_plane"]["control"]["CD,daileron"])<1e-10
    assert abs(derivs["test_plane"]["control"]["CS,daileron"]-0.13409255196609768)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cx,daileron"])<1e-10
    assert abs(derivs["test_plane"]["control"]["Cy,daileron"]-0.13409255196609768)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cz,daileron"])<1e-10
    assert abs(derivs["test_plane"]["control"]["Cl,daileron"]+0.4829213183114422)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cm,daileron"])<1e-10
    assert abs(derivs["test_plane"]["control"]["Cn,daileron"]+0.1173202524037608)<1e-10
    assert abs(derivs["test_plane"]["control"]["CL,delevator"]-1.6795004768670678)<1e-10
    assert abs(derivs["test_plane"]["control"]["CD,delevator"]-0.030957629339379567)<1e-10
    assert abs(derivs["test_plane"]["control"]["CS,delevator"]+2.7500552332993075e-16)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cx,delevator"]-0.02767495056623802)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cy,delevator"]+2.7500552332993075e-16)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cz,delevator"]+1.6795577762381937)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cl,delevator"])<1e-10
    assert abs(derivs["test_plane"]["control"]["Cm,delevator"]+4.975416426126367)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cn,delevator"]-1.5931710494786176e-16)<1e-10
    assert abs(derivs["test_plane"]["control"]["CL,drudder"])<1e-10
    assert abs(derivs["test_plane"]["control"]["CD,drudder"])<1e-10
    assert abs(derivs["test_plane"]["control"]["CS,drudder"]+0.6384357546253706)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cx,drudder"])<1e-10
    assert abs(derivs["test_plane"]["control"]["Cy,drudder"]+0.6384357546253706)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cz,drudder"])<1e-10
    assert abs(derivs["test_plane"]["control"]["Cl,drudder"]+0.1411603229331166)<1e-10
    assert abs(derivs["test_plane"]["control"]["Cm,drudder"])<1e-10
    assert abs(derivs["test_plane"]["control"]["Cn,drudder"]-0.5111158114523209)<1e-10