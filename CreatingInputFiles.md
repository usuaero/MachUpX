# Creating Input Files for MachUpX
This document is meant to be a complete reference for creating MachUp input files.
The basic input file for MachUp is a .json file which describes the scene and which
aircraft are in the scene, along with their state in the scene. A separate .json
file is used to specify the geometry and controls of each aircraft. These aircraft
files can reference other files that store information on airfoil properties, chord
distribution, sweep, etc., as will be discussed. The very minimum files required are
a file describing the scene and a file describing the one aircraft in the scene. 
Only the scene file is passed to MachUp as an argument or used as the input in the
API, as it references all necessary aircraft files.

# Scene File
The following are tags which can be specified in the scene .json object. NOTE: some
are required, others are optional.

"tag" : (string, optional)
    A note on the specific input.

"run" : (dict, optional)
    Gives the analyses MachUp is to run. The following keys are available:

    "forces"

    "stl"

    "aero_derivatives"

    "state_derivatives"

    "distributions"
    
    This must be specified if the input file is being passed as a command line argument
    to MachUp. Otherwise, MachUp will do nothing. If this input file is being given to
    the API, this does not affect execution.

"solver" : (dict, optional)
    Specifies parameters regarding how the lifting-line equation is solved. The following
    keys are available:

    "type" : (string)
        Can be "linear" or "nonlinear". Defaults to "linear".

    "convergence" : (float)
        Defaults to 1e-10

    "relaxation" : (float)
        Defaults to 0.9.