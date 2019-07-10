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
The following are tags which can be specified in the scene .json object. NOTE: all keys
not marked as optional are required.

    "tag" : (string, optional)
        A note on the specific input.

    "run" : (dict, optional)
        Gives the analyses MachUp is to run. The following keys are available:

        "forces" : ()

        "stl" : ()

        "aero_derivatives" : ()

        "state_derivatives" : ()

        "distributions" : ()

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

    "units" : (string)
        Specifies the units to be used for inputs and outputs. Can be "SI" or "English".

    "scene" : (dict)

        "atmosphere" : (dict)
            Specifies the state of the atmosphere the aircraft exist in.

            "rho" : (float, 2D array, or "file")
                If a float, the atmospheric density is assumed constant. If a 2D array is given,
                this is assumed to be a density profile of the atmosphere. The first column is
                heights and the second column is densities. MachUp will linearly interpolate these
                data. If specified as "file", MachUp will pull the densities from the filename
                given by "path" (see below).
            
            "V_wind" : (1D array or "file")
                If a 1D array is given, this is assumed to be the wind velocity vector given in
                flat-earth coordinates which is constant throughout the scene. If specified as 
                "file", MachUp will pull the local velocity from the filename given by "path"
                (see below).

            "path" : (string)
                Path to text file containing densities and velocities as a function of position.
                The first three columns give the position in flat-earth coordinates, the fourth
                column gives the air density, the fifth through seventh columns give the local wind
                vector in earth-fixed coordinates.

        "aircraft" : (dict)
            Lists the aircraft to be placed in the scene. At least one must be specified. If importing
            more than one aircraft, simply repeat the following:

            "<AIRPLANE NAME>" : (dict)

                "file" : (string)
                    Path to file containing the .json object describing the airplane.

                "state" : (dict)
                    Describes the state of the aircraft.

                    "type" : (string)
                        Specifies which definition of state is to be used. If given as "rigid_body", the
                        following keys must be specified within "state" (not as part of "type"):

                    "position" : (1D array)
                        Position of the origin of the aircraft's body-fixed coordinate system in flat-earth
                        coordinates.

                    "velocity" : (1D array)
                        Velocity of the aircraft in flat-earth coordinates.

                    "orientation" : (1D array)
                        Orientation of the aircraft in flat-earth coordinates. If this is a 3-element array,
                        it is assumed an Euler angle formulation is used. If this is a 4-element array, it is
                        assumed a quaternion formulation is used.

                    "rates" : (1D array)
                        Angular rate of the aircraft in flat-earth coordinates.

                    If "type" is specified as "aerodynamic", the following keys must be specified:

                    "position" : (1D array)
                        Position of the origin of the aircraft's body-fixed coordinate system in flat-earth
                        coordinates.

                    "rates" : (1D array)
                        Angular rate of the aircraft in flat-earth coordinates.

                    "V_mag" : (float)
                        Magnitude of the local wind vector.

                    "alpha" : (float)
                        Aerodynamic angle of attack.

                    "beta" : (float)
                        Aerodynamic sideslip angle.