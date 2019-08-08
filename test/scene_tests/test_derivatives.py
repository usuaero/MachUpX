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

    assert abs(stab_derivs["CL,a"]-6.325561939795834)<1e-10
    assert abs(stab_derivs["CD,a"]-0.32484784717545706)<1e-10
    assert abs(stab_derivs["CS,a"])<1e-10
    assert abs(stab_derivs["Cx,a"]-0.11701146202557859)<1e-10
    assert abs(stab_derivs["Cy,a"])<1e-10
    assert abs(stab_derivs["Cz,a"]+6.340112485605566)<1e-10
    assert abs(stab_derivs["Cl,a"])<1e-10
    assert abs(stab_derivs["Cm,a"]+3.951172935738557)<1e-10
    assert abs(stab_derivs["Cn,a"])<1e-10
    assert abs(stab_derivs["CL,B"])<1e-10
    assert abs(stab_derivs["CD,B"])<1e-10
    assert abs(stab_derivs["CS,B"]+0.8173565005198824)<1e-10
    assert abs(stab_derivs["Cx,B"])<1e-10
    assert abs(stab_derivs["Cy,B"]+0.8320251585209351)<1e-10
    assert abs(stab_derivs["Cz,B"])<1e-10
    assert abs(stab_derivs["Cl,B"]+0.13833770562075048)<1e-10
    assert abs(stab_derivs["Cm,B"])<1e-10
    assert abs(stab_derivs["Cn,B"]-0.6187734326780242)<1e-10


def test_damping_derivatives():
    # Test the calculation of the damping derivatives

    # Load scene
    scene = MX.Scene(input_file)
    damp_derivs = scene.aircraft_damping_derivatives()

    # There's something about how NLL calculates these cases that introduces a little more numerical error...
    assert abs(damp_derivs["CL,pbar"]-0.00037018702894742184)<1e-6
    assert abs(damp_derivs["CD,pbar"]+6.60883695078468e-05)<1e-6
    assert abs(damp_derivs["CS,pbar"]+0.09305320429644524)<1e-6
    assert abs(damp_derivs["Cx,pbar"]-7.896745125857141e-05)<1e-6
    assert abs(damp_derivs["Cy,pbar"]+0.09305320429644524)<1e-6
    assert abs(damp_derivs["Cz,pbar"]+0.0003676550701381398)<1e-6
    assert abs(damp_derivs["Cl,pbar"]+2.55335879459208)<1e-6
    assert abs(damp_derivs["Cm,pbar"]+0.0011593362887751812)<1e-6
    assert abs(damp_derivs["Cn,pbar"]-0.02514714847131455)<1e-6
    assert abs(damp_derivs["CL,qbar"]-13.019918503532345)<1e-6
    assert abs(damp_derivs["CD,qbar"]-0.3090180115277938)<1e-6
    assert abs(damp_derivs["CS,qbar"]+0.013965339163743515)<1e-6
    assert abs(damp_derivs["Cx,qbar"]-0.14555883677632234)<1e-6
    assert abs(damp_derivs["Cy,qbar"]+0.013965339163743515)<1e-6
    assert abs(damp_derivs["Cz,qbar"]+13.022771694040092)<1e-6
    assert abs(damp_derivs["Cl,qbar"]+0.0011967792488973804)<1e-6
    assert abs(damp_derivs["Cm,qbar"]+36.6535222940767)<1e-6
    assert abs(damp_derivs["Cn,qbar"]-0.010480645283789216)<1e-6
    assert abs(damp_derivs["CL,rbar"]+2.211564265053312e-09)<1e-6
    assert abs(damp_derivs["CD,rbar"]+4.9465639917478654e-11)<1e-6
    assert abs(damp_derivs["CS,rbar"]-1.2322179506649182)<1e-6
    assert abs(damp_derivs["Cx,rbar"]+2.776858604169874e-11)<1e-6
    assert abs(damp_derivs["Cy,rbar"]-1.2322179506649182)<1e-6
    assert abs(damp_derivs["Cz,rbar"]-2.2119805986875463e-09)<1e-6
    assert abs(damp_derivs["Cl,rbar"]-0.36543638451918914)<1e-6
    assert abs(damp_derivs["Cm,rbar"]-6.248890294102694e-09)<1e-6
    assert abs(damp_derivs["Cn,rbar"]+0.9323904254656564)<1e-6


def test_control_derivatives():
    # Test the calculation of the control derivatives

    # Load scene
    scene = MX.Scene(input_file)
    cont_derivs = scene.aircraft_control_derivatives()

    assert abs(cont_derivs["CL,daileron"])<1e-10
    assert abs(cont_derivs["CD,daileron"])<1e-10
    assert abs(cont_derivs["CS,daileron"]-0.13409255196609768)<1e-10
    assert abs(cont_derivs["Cx,daileron"])<1e-10
    assert abs(cont_derivs["Cy,daileron"]-0.13409255196609768)<1e-10
    assert abs(cont_derivs["Cz,daileron"])<1e-10
    assert abs(cont_derivs["Cl,daileron"]+0.4829213183114422)<1e-10
    assert abs(cont_derivs["Cm,daileron"])<1e-10
    assert abs(cont_derivs["Cn,daileron"]+0.1173202524037608)<1e-10
    assert abs(cont_derivs["CL,delevator"]-1.6795004768670678)<1e-10
    assert abs(cont_derivs["CD,delevator"]-0.030957629339379567)<1e-10
    assert abs(cont_derivs["CS,delevator"]+2.7500552332993075e-16)<1e-10
    assert abs(cont_derivs["Cx,delevator"]-0.02767495056623802)<1e-10
    assert abs(cont_derivs["Cy,delevator"]+2.7500552332993075e-16)<1e-10
    assert abs(cont_derivs["Cz,delevator"]+1.6795577762381937)<1e-10
    assert abs(cont_derivs["Cl,delevator"])<1e-10
    assert abs(cont_derivs["Cm,delevator"]+4.975416426126367)<1e-10
    assert abs(cont_derivs["Cn,delevator"]-1.5931710494786176e-16)<1e-10
    assert abs(cont_derivs["CL,drudder"])<1e-10
    assert abs(cont_derivs["CD,drudder"])<1e-10
    assert abs(cont_derivs["CS,drudder"]+0.6384357546253706)<1e-10
    assert abs(cont_derivs["Cx,drudder"])<1e-10
    assert abs(cont_derivs["Cy,drudder"]+0.6384357546253706)<1e-10
    assert abs(cont_derivs["Cz,drudder"])<1e-10
    assert abs(cont_derivs["Cl,drudder"]+0.1411603229331166)<1e-10
    assert abs(cont_derivs["Cm,drudder"])<1e-10
    assert abs(cont_derivs["Cn,drudder"]-0.5111158114523209)<1e-10


def test_all_derivs():
    # Test the calculation of the derivatives

    # Load scene
    scene = MX.Scene(input_file)
    derivs = scene.aircraft_derivatives()
    assert abs(derivs["stability"]["CL,a"]-6.325561939795834)<1e-10
    assert abs(derivs["stability"]["CD,a"]-0.32484784717545706)<1e-10
    assert abs(derivs["stability"]["CS,a"])<1e-10
    assert abs(derivs["stability"]["Cx,a"]-0.11701146202557859)<1e-10
    assert abs(derivs["stability"]["Cy,a"])<1e-10
    assert abs(derivs["stability"]["Cz,a"]+6.340112485605566)<1e-10
    assert abs(derivs["stability"]["Cl,a"])<1e-10
    assert abs(derivs["stability"]["Cm,a"]+3.951172935738557)<1e-10
    assert abs(derivs["stability"]["Cn,a"])<1e-10
    assert abs(derivs["stability"]["CL,B"])<1e-10
    assert abs(derivs["stability"]["CD,B"])<1e-10
    assert abs(derivs["stability"]["CS,B"]+0.8173565005198824)<1e-10
    assert abs(derivs["stability"]["Cx,B"])<1e-10
    assert abs(derivs["stability"]["Cy,B"]+0.8320251585209351)<1e-10
    assert abs(derivs["stability"]["Cz,B"])<1e-10
    assert abs(derivs["stability"]["Cl,B"]+0.13833770562075048)<1e-10
    assert abs(derivs["stability"]["Cm,B"])<1e-10
    assert abs(derivs["stability"]["Cn,B"]-0.6187734326780242)<1e-10
    assert abs(derivs["damping"]["CL,pbar"]-0.00037018702894742184)<1e-6
    assert abs(derivs["damping"]["CD,pbar"]+6.60883695078468e-05)<1e-6
    assert abs(derivs["damping"]["CS,pbar"]+0.09305320429644524)<1e-6
    assert abs(derivs["damping"]["Cx,pbar"]-7.896745125857141e-05)<1e-6
    assert abs(derivs["damping"]["Cy,pbar"]+0.09305320429644524)<1e-6
    assert abs(derivs["damping"]["Cz,pbar"]+0.0003676550701381398)<1e-6
    assert abs(derivs["damping"]["Cl,pbar"]+2.55335879459208)<1e-6
    assert abs(derivs["damping"]["Cm,pbar"]+0.0011593362887751812)<1e-6
    assert abs(derivs["damping"]["Cn,pbar"]-0.02514714847131455)<1e-6
    assert abs(derivs["damping"]["CL,qbar"]-13.019918503532345)<1e-6
    assert abs(derivs["damping"]["CD,qbar"]-0.3090180115277938)<1e-6
    assert abs(derivs["damping"]["CS,qbar"]+0.013965339163743515)<1e-6
    assert abs(derivs["damping"]["Cx,qbar"]-0.14555883677632234)<1e-6
    assert abs(derivs["damping"]["Cy,qbar"]+0.013965339163743515)<1e-6
    assert abs(derivs["damping"]["Cz,qbar"]+13.022771694040092)<1e-6
    assert abs(derivs["damping"]["Cl,qbar"]+0.0011967792488973804)<1e-6
    assert abs(derivs["damping"]["Cm,qbar"]+36.6535222940767)<1e-6
    assert abs(derivs["damping"]["Cn,qbar"]-0.010480645283789216)<1e-6
    assert abs(derivs["damping"]["CL,rbar"]+2.211564265053312e-09)<1e-6
    assert abs(derivs["damping"]["CD,rbar"]+4.9465639917478654e-11)<1e-6
    assert abs(derivs["damping"]["CS,rbar"]-1.2322179506649182)<1e-6
    assert abs(derivs["damping"]["Cx,rbar"]+2.776858604169874e-11)<1e-6
    assert abs(derivs["damping"]["Cy,rbar"]-1.2322179506649182)<1e-6
    assert abs(derivs["damping"]["Cz,rbar"]-2.2119805986875463e-09)<1e-6
    assert abs(derivs["damping"]["Cl,rbar"]-0.36543638451918914)<1e-6
    assert abs(derivs["damping"]["Cm,rbar"]-6.248890294102694e-09)<1e-6
    assert abs(derivs["damping"]["Cn,rbar"]+0.9323904254656564)<1e-6
    assert abs(derivs["control"]["CL,daileron"])<1e-10
    assert abs(derivs["control"]["CD,daileron"])<1e-10
    assert abs(derivs["control"]["CS,daileron"]-0.13409255196609768)<1e-10
    assert abs(derivs["control"]["Cx,daileron"])<1e-10
    assert abs(derivs["control"]["Cy,daileron"]-0.13409255196609768)<1e-10
    assert abs(derivs["control"]["Cz,daileron"])<1e-10
    assert abs(derivs["control"]["Cl,daileron"]+0.4829213183114422)<1e-10
    assert abs(derivs["control"]["Cm,daileron"])<1e-10
    assert abs(derivs["control"]["Cn,daileron"]+0.1173202524037608)<1e-10
    assert abs(derivs["control"]["CL,delevator"]-1.6795004768670678)<1e-10
    assert abs(derivs["control"]["CD,delevator"]-0.030957629339379567)<1e-10
    assert abs(derivs["control"]["CS,delevator"]+2.7500552332993075e-16)<1e-10
    assert abs(derivs["control"]["Cx,delevator"]-0.02767495056623802)<1e-10
    assert abs(derivs["control"]["Cy,delevator"]+2.7500552332993075e-16)<1e-10
    assert abs(derivs["control"]["Cz,delevator"]+1.6795577762381937)<1e-10
    assert abs(derivs["control"]["Cl,delevator"])<1e-10
    assert abs(derivs["control"]["Cm,delevator"]+4.975416426126367)<1e-10
    assert abs(derivs["control"]["Cn,delevator"]-1.5931710494786176e-16)<1e-10
    assert abs(derivs["control"]["CL,drudder"])<1e-10
    assert abs(derivs["control"]["CD,drudder"])<1e-10
    assert abs(derivs["control"]["CS,drudder"]+0.6384357546253706)<1e-10
    assert abs(derivs["control"]["Cx,drudder"])<1e-10
    assert abs(derivs["control"]["Cy,drudder"]+0.6384357546253706)<1e-10
    assert abs(derivs["control"]["Cz,drudder"])<1e-10
    assert abs(derivs["control"]["Cl,drudder"]+0.1411603229331166)<1e-10
    assert abs(derivs["control"]["Cm,drudder"])<1e-10
    assert abs(derivs["control"]["Cn,drudder"]-0.5111158114523209)<1e-10