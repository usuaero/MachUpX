# Creating Input Files for MachUpX
This document is meant to be a complete reference for creating MachUp input files.
The basic input file for MachUp contains a JSON object which describes the scene and which
aircraft are in the scene, along with the state of those aircraft in the scene. A separate JSON 
object is used to specify the geometry and controls of each aircraft. These aircraft
objects can reference other files that store information on airfoil properties, chord
distributions, sweep, etc., as will be discussed. At a minimum, two JSON objects must be specified,
a scene object and an aircraft object.

# Input Format
The basic structure of a JSON object is a set of key-value pairs, analogous to a Python dict. See the
example input files for an example of the exact format. The following sections describe the structure
of the JSON objects used to interface with MachUp. Only one JSON object is specified per .json file.
When using the JSON objects, only the scene object is passed to MachUp. As long as the paths to all
other JSON objects are properly specified in the scene object, MachUp will automatically load all other
required objects.

MachUp allows the user to specify the units for each value if they desire. For float values, this is done
by making the vaule a list where the first element is the actual value and the second element is a string
specifying the units. For example:

```python
"area" : [6.75,"ft^2"]
```

For vector inputs, such as position and velocity, the units are simply appended to the vector:

```python
"velocity" : [100,13,0,"ft/s"]
```

For array inputs, such as a density profile or chord distribution, the units are appended in the first dimension:

```python
"rho" : [[0.0,1.225],
         [2000.0,1.0066],
         [4000.0,0.81935],
         "kg/m^3"]
```

Each key here lists which units are allowed.

## Scene Object
The following are tags which can be specified in the scene JSON object. NOTE: all keys not marked as
optional are required. Key values typed in all capitals between carats (e.g. <KEY_VALUE>) are to be
deterimined by the user.

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
        Specifies the units to be used for inputs and outputs. Can be "SI" or "English". Any units
        not explicitly defined for each value in the input objects will be assumed to be the standard
        unit for that measurement in the system specified here.

    "scene" : (dict)

        "atmosphere" : (dict)
            Specifies the state of the atmosphere the aircraft exist in.

            "rho" : (float, array, or string)
                If a float, the atmospheric density is assumed constant. If a 2D array is given,
                this is assumed to be a density profile of the atmosphere. The first column is
                heights and the second column is densities. MachUp will linearly interpolate these
                data. If specified as "file", MachUp will pull the densities from the filename
                given by "path" (see below). The user can also specify this as "standard", in which
                case a standard atmosphere profile will be used.
                Allowable units: "slug/ft^3", "kg/m^3"
            
            "V_wind" : (vector or "file")
                If a 1D array is given, this is assumed to be the wind velocity vector given in
                flat-earth coordinates which is constant throughout the scene. If specified as 
                "file", MachUp will pull the local velocity from the filename given by "path"
                (see below). NOTE: This value is optional if the state type of all aircraft is
                specified as "aerodynamic".
                Allowable units: "ft/s", "m/s", "mph", "kph", "kn"

            "path" : (string)
                Path to text file containing densities and velocities as a function of position.
                The first three columns give the position in flat-earth coordinates, the fourth
                column gives the air density, the fifth through seventh columns give the local wind
                vector in earth-fixed coordinates.

        "aircraft" : (dict)
            Lists the aircraft to be placed in the scene. At least one must be specified. If importing
            more than one aircraft, simply repeat the following:

            "<AIRPLANE_NAME>" : (dict)

                "file" : (string)
                    Path to file containing the JSON object describing the airplane.

                "state" : (dict)
                    Describes the state of the aircraft.

                    "type" : (string)
                        Specifies which definition of state is to be used. If given as "rigid_body", the
                        following keys must be specified within "state" (not as part of "type"):

                    "position" : (vector)
                        Position of the origin of the aircraft's body-fixed coordinate system in flat-earth
                        coordinates.
                        Allowable units: "ft", "m"

                    "velocity" : (vector)
                        Velocity of the aircraft in flat-earth coordinates.
                        Allowable units: "ft/s", "m/s", "mph", "kph", "kn"

                    "orientation" : (vector)
                        Orientation of the aircraft in flat-earth coordinates. If this is a 3-element vector
                        it is assumed an Euler angle formulation is used. If this is a 4-element vector it is
                        assumed a quaternion formulation is used.
                        Allowable units: "deg", "rad" (for Euler; quaternion is nondimensional)

                    "rates" : (vector)
                        Angular rate of the aircraft in flat-earth coordinates.
                        Allowable units: "deg/s", "rad/s"

                    If "type" is specified as "aerodynamic", the following keys must be specified:

                    "position" : (vector)
                        Position of the origin of the aircraft's body-fixed coordinate system in flat-earth
                        coordinates.
                        Allowable units: "ft", "m"

                    "rates" : (vector)
                        Angular rate of the aircraft in flat-earth coordinates.
                        Allowable units: "deg/s", "rad/s"

                    "V_mag" : (float)
                        Magnitude of the local wind vector.
                        Allowable units: "ft/s", "m/s", "mph", "kph", "kn"

                    "alpha" : (float)
                        Aerodynamic angle of attack.
                        Allowable units: "deg", "rad"

                    "beta" : (float)
                        Aerodynamic sideslip angle.
                        Allowable units: "deg", "rad"

                "control_state" : (dict, optional)
                    Describes the control deflections. The number and names of controls are arbitrary and may be
                    specified by the user. This is discussed more in depth as part of the aircraft object. If the
                    aircraft has controls but no state is specified, all deflections will be assumed to be zero.

                    "<CONTROL_NAME>" : (dict)

                        "deflection" : (float)
                            Control surface deflection.
                            Allowable units: "deg", "rad"

## Aircraft Object
Describes an aircraft.

    "name" : (string)
        The name of the aircraft.

    "CG" : (1D array)
        Location of the aircraft's center of gravity in body-fixed coordinates. For simplicity, the CG is
        often assumed to be coincident with the body-fixed origin.

    "reference" : (dict, optional)
        Specifies the reference lengths and areas used for nondimensional analysis. Any or none of these may be specified. If not specified, MachUp will select appropriate values based on the geometry of the main wing.

        "area" : (float)
            The reference area.
            Allowable units: "ft^2", "in^2", "m^2", or "cm^2".

        "longitudinal_length" : (float)
            Longitudinal reference length.
            Allowable units: "ft", "in", "m", or "cm".

        "lateral_length" : (float)
            Lateral reference length.
            Allowable units: "ft", "in", "m", or "cm".

    "controls" : (list, optional)
        Defines the controls of the aircraft. The number and names of controls are arbitrary and may be specified
        by the user. A simple aircraft, such as a chuck glider may have no controls, whereas a more complex
        aircraft may have controls for aileron, elevator, rudder, throttle, and multiple flaps. Defining the
        controls here can be thought of as deciding which control knobs/switches/sticks you want to make
        available to the pilot. Later in the object, when defining the wings, you will define how each control
        affects the deflection of each control surface.

    "airfoils" : (dict)
        Defines the airfoil section parameters for all airfoils used on the aircraft. A dict defining an airfoil
        has the following structure:

        "<AIRFOIL_NAME>" : (dict)

            "type" : (string)
                The type of information describing the airfoil. If "linear", the following keys are required:

            "alpha_L0" : (float)
                The zero-lift angle of attack in radians.

            "CL_alpha" : (float)
                The lift slope in radians^-1.

            "Cm_L0" : (float)
                The zero-lift moment coefficient.

            "Cm_alpha" : (float)
                The moment slope in radians^-1.

            "CD0" : (float)
                Constant coefficient in the quadratic fit of the CD/CL curve.

            "CD_L" : (float)
                Linear coefficient in the quadratic fit of the CD/CL curve.

            "CD_L2" : (float)
                Quadratic coefficient in the quadratic fir of the CD/CL curve.

            "CL_max" : (float)
                Maximum lift coefficient.

            If "type" is "file", the following key is required:

            "path" : (string)
                Path to file containing either a JSON object describing the airfoil formatted as above or tabulated
                data of airfoil coefficients as a function of angle of attack and Reynolds number (described 
                below).

        Any number of airfoils can be defined for the aircraft simply by repeating the above structure within the
        "airfoils" dict. MachUp pulls from these airfoil definitions as needed, depending on which airfoils are
        specified for the wings.

    "wings" : (dict)
        Gives the lifting surfaces for the aircraft. Wings, stabilizers, fins, etc. are all treated the same in
        numerical lifting-line and so should be included here as wings.