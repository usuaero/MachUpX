# Creating Input Files for MachUpX
This document is meant to be a complete reference for creating MachUp input files. The basic input file for 
MachUp contains a JSON object which describes the scene and which aircraft are in the scene, along with the 
state of those aircraft in the scene. A separate JSON object is used to specify the geometry and controls 
of each aircraft. These aircraft objects can reference other files that store information on airfoil 
properties, chord distributions, sweep, etc., as will be discussed. At a minimum, two JSON objects must be 
specified, a scene object and an aircraft object.

## Input Format
The basic structure of a JSON object is a set of key-value pairs, analogous to a Python dict. See the
example input files for an example of the exact format. The following sections describe the structure
of the JSON objects used to interface with MachUp. Only one JSON object is specified per .json file.
When using the JSON objects, only the scene object is passed to MachUp. As long as the paths to all
other JSON objects are properly specified in the scene object, MachUp will automatically load all other
required objects.

Boolean values are encoded as a 1 or a 0.

### Units

MachUp allows the user to specify the units for each value if they desire. For float values, this is done
by making the value a list where the first element is the actual value and the second element is a string
specifying the units. For example:

```python
"area" : [6.75,"ft^2"]
```

For vector inputs, such as position and velocity, the units are simply appended to the vector:

```python
"velocity" : [100,13,0,"ft/s"]
```

For array inputs, such as a density profile or chord distribution, the units are appended as another row
in the array:

```python
"rho" : [[0.0,1.225],
         [2000.0,1.0066],
         [4000.0,0.81935],
         ["m","kg/m^3"]]
```

When specifying column units in files, these are prepended as another row:

```python
# File: density_profile.csv
"m", "kg/m^3",
0.0,1.225,
2000.0,1.0066,
4000.0,0.81935
```

The following measurements can be defined with the accompanying units:

| Measurement                   | MachUp Units                      |
|-------------------------------|-----------------------------------|
| Position/displacement/length: | "ft", "m", "in", "cm"             |
| Area:                         | "ft^2", "m^2"                     |
| Velocity:                     | "ft/s", "m/s", "mph", "kph", "kn" |
| Angular deflection/position:  | "deg", "rad"                      |
| Angular rate:                 | "deg/s", "rad/s"                  |
| Density:                      | "slug/ft^3", "kg/m^3"             |
| Weight:                       | "lbf", "N"                        |

Except for angular measurements, the first unit listed is the default for "English" units. The second is 
the default for "SI". For measurements of state and control deflections, the default is always degrees. 
For airfoil parameters, measurements may only be expressed in radians. When specifying units in an array 
and one of the measurements is dimensionless (e.g. chord fraction), "-" should be used.

### Scene Object
The following are keys which can be specified in the scene JSON object. NOTE: all keys not marked as
optional are required. Key names typed in all capitals between carats (e.g. <KEY_VALUE>) are to be
deterimined by the user.

    "tag" : (string, optional)
        A note on the specific input. Does not affect execution.

    "run" : (dict, optional)
        Gives the analyses MachUp is to run. This must be specified if the input file is being passed 
        as a command line argument to MachUp. Otherwise, MachUp will return without performing any 
        calculations. If this input file is being given to the API, this does not affect execution.

        "forces" : (dict, optional)
            Calculates the aerodynamic forces and moments on the aircraft at the current state.

            "output_file" : (string, optional)
                File to store the results in. Defaults to the input filename + "_forces".

            "dimensional" : (boolean, optional)
                Whether results should be returned as dimensional or nondimensional. If 1, will return
                forces and moments in "lbf" and "ft-lbf" or "N" and "Nm", depending on how "units" is 
                specified. Defaults to 1.

        "stl" : ()
            Outputs an stl file of the geometry of the scene/aircraft.

        "igs" : ()
            Outputs an igs file of the geometry of the scene/aircraft.

        "aero_derivatives" : ()

        "state_derivatives" : ()

        "distributions" : ()

    "solver" : (dict, optional)
        Specifies parameters regarding how the lifting-line equation is solved.

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

            "rho" : (float, array, or string, optional)
                If a float, the atmospheric density is assumed constant. If an array is given,
                this is assumed to be a density profile of the atmosphere. The first column is
                heights and the second column is densities. MachUp will linearly interpolate these
                data. The user can also specify this as "standard", in which case a standard atmosphere 
                profile will be used. This can also be a path to a csv file containing the density as
                a function of position. The first three columns contain the position in flat-earth 
                coordinates and the last column contains the density. The file can also contain 
                information in the same form as the profile array. Defaults to density at sea-level.
            
            "V_wind" : (vector or string, optional)
                If a vector is given, this is assumed to be the wind velocity vector given in
                flat-earth coordinates which is constant throughout the scene. If a string is given,
                this is the path to a csv file containing the wind velocity as a function of position.
                The first three columns contain the position and the last three columns contain the 
                velocity vector at each position, all in flat-earth coordinates. NOTE: This does not affect 
                computation for all aircraft where the "type" of the "state" is set as "aerodynamic".
                Defaults to no wind.

        "aircraft" : (dict)
            Lists the aircraft to be placed in the scene. At least one must be specified. If importing
            more than one aircraft, simply repeat the following:

            "<AIRPLANE_NAME>" : (dict)

                "file" : (string)
                    Path to file containing the JSON object describing the aircraft.

                "state" : (dict)
                    Describes the state of the aircraft.

                    "type" : (string)
                        Specifies which definition of state is to be used. If given as "rigid_body", then "position", "velocity", "orientation", and "angular_rates" can be  specified as 
                        well. If given as "aerodynamic", then "position", "velocity", "angular_rates", 
                        "alpha", and "beta" can be specified as well. Specifying both "rigid_body" and 
                        "aerodynamic" state variables will throw an input error.

                    "position" : (vector, optional)
                        Position of the origin of the aircraft's body-fixed coordinate system in flat-earth
                        coordinates. Defaults to [0,0,0]

                    "velocity" : (float or vector)
                        In the case of "type" = "rigid_body":
                            Velocity vector of the aircraft in flat-earth coordinates. Cannot be float.

                        In the case of "type" = "aerodynamic":
                            If a float, this is the magnitude of the local wind vector. If a vector, this 
                            is the local freestream vector (i.e. u, v, and w). Specifying a velocity vector 
                            will override definitions for "alpha" and "beta".

                    "orientation" : (vector, optional)
                        Orientation of the aircraft in flat-earth coordinates. If this is a 3-element 
                        vector it is assumed the Euler angle formulation is used (i.e. [phi, theta, psi]).  
                        If this is a 4-element vector it is assumed the quaternion formulation is used. 
                        Defaults to [1.0, 0.0, 0.0, 0.0].

                    "angular_rates" : (vector, optional)
                        Angular rate of the aircraft in flat-earth coordinates. Defaults to [0,0,0]

                    "alpha" : (float, optional)
                        Aerodynamic angle of attack. Defaults to 0.

                    "beta" : (float, optional)
                        Aerodynamic sideslip angle. Defaults to 0. NOTE: MachUp defines this as the 
                        analytical sideslip angle, i.e. B = atan(Vy/Vx).

                "control_state" : (dict, optional)
                    Describes the control deflections. The number and names of controls are arbitrary and 
                    may be specified by the user. This is discussed more in depth as part of the aircraft 
                    object. If the aircraft has controls but no state is specified, all deflections will 
                    be assumed to be zero.

                    "<CONTROL_NAME>" : (float)
                        Control surface deflection.

### Aircraft Object
Describes an aircraft.

    "name" : (string)
        The name of the aircraft.

    "CG" : (vector, optional)
        Location of the aircraft's center of gravity in body-fixed coordinates. Defaults to [0,0,0].

    "weight" : (float)
        Weight of the aircraft.

    "reference" : (dict, optional)
        Specifies the reference lengths and areas used for nondimensional analysis. Any or none of these 
        may be specified. If not specified, MachUp will select appropriate values based on the geometry of 
        the main wing.

        "area" : (float)
            The reference area.

        "longitudinal_length" : (float)
            Longitudinal reference length.

        "lateral_length" : (float)
            Lateral reference length.

    "controls" : (list, optional)
        Defines the controls of the aircraft. The number and names of controls are arbitrary and may be 
        specified by the user. A simple aircraft, such as a chuck glider may have no controls, whereas a 
        more complex aircraft may have controls for aileron, elevator, rudder, throttle, and multiple 
        flaps. Defining the controls here can be thought of as deciding which control knobs/switches/
        sticks you want to make available to the pilot. Later in the object, when defining the wings, you 
        will define how each control affects the deflection of each control surface.

    "airfoils" : (dict)
        Defines the airfoil section parameters for all airfoils used on the aircraft. Any number of 
        airfoils can be defined for the aircraft. MachUp pulls from these airfoil definitions as needed, 
        depending on which airfoils are specified for the wings. A dict defining an airfoil has the 
        following structure:

        "<AIRFOIL_NAME>" : (dict)

            "type" : (string)
                The type of information describing the airfoil. Can be "linear" or "nonlinear".If 
                "nonlinear", then "path" must give the location of an airfoil data file as described
                in the next section of this document. If "linear", the following keys must be defined,
                either here or in a JSON object pointed to by "path".

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

            "path" : (string, optional)
                Path to file containing either a JSON object describing the airfoil using the above keys 
                or tabulated data of airfoil coefficients as a function of angle of attack and Reynolds 
                number (described as part of the following key).

            "generate_database" : (bool, optional)
                If you do not know the properties of the airfoil you would like to use, setting this to 
                1 will instruct MachUp to model the airfoil and automatically generate the required 
                data. This can be done one of two ways. If "<AIRFOIL_NAME>" is "NACA_" followed by 4 or 
                5 digits, MachUp will generate the geometry according to the NACA equations. Alternatively,
                "path" can be set to point to a text file containing a series of x-y points describing 
                the geometry of the airfoil. This file should be formatted in two columns, separated by
                spaces, with the x-coordinate in the first column and the y-coordinate in the second 
                column. The resulting data will be stored in a file named after the airfoil according to 
                which "type" is specified. Defaults to 0.

    "wing_segments" : (dict)
        Gives the lifting surfaces for the aircraft. Wings, stabilizers, fins, etc. are all treated the 
        same in numerical lifting-line and so should be included here as wing segments. MachUp is set up so 
        the user can define complex geometries by attaching the ends of different wing segments together 
        (for an example, see the /examples directory). The user can define any number of wing segments 
        within this dict.

        "<WING_SEGMENT_NAME>" : (dict)

            "name" : (string)

            "ID" : (uint)
                ID tag of the wing segment, used for specifying which other wing segments are defined 
                relative to it. May not be 0.

            "is_main" : (bool)
                Specifies whether this wing segment is part of the main wing (used for determining
                reference lengths and areas).

            "side" : (string)
                May be "right", "left", or "both". Defines which side(s) of the aircraft the wing segment
                appears on. If "both", the wing segment will be mirrored across the x-z plane.

            "connect_to" : (dict)
                Places the origin for the wing segment. This can be defined relative to the aircraft's
                body-fixed origin, or the root or tip of any other wing segment.

                "ID" : (uint, optional)
                    ID of the wing segment this wing segment's origin is being defined relative to. If 0,
                    this wing segment's origin will be defined relative to the aircraft's body-fixed 
                    origin. Defaults to 0.

                "location" : (string, optional)
                    May be "root" or "tip". Defines whether this wing segment's origin should be defined 
                    relative to the root or tip of the other wing segment. Defaults to "tip"

                "dx" : (float, optional)
                    Displacement of the origin from the selected reference point in the body-fixed x-
                    direction. Defaults to 0.

                "dy" : (float, optional)
                    Displacement of the origin from the selected reference point in the body-fixed y-
                    direction. NOTE: If "side" is specified as "both", changing this value will shift
                    both sides of the wing segment in the SAME direction. The effect is not mirrored.
                    Defaults to 0.

                "dz" : (float, optional)
                    Displacement of the origin from the selected reference point in the body-fixed z-
                    direction. Defaults to 0.

                "y_offset" : (float, optional)
                    Distance the origin should be shifted from the centerline (positive offset 
                    corresponds to outward). If "side" is specified as "both", this effect is mirrored.
                    Defaults to 0.
            
            "span" : (float)
                Length of the wing segment, discounting sweep. If "side" is specified as "both", the total
                span of the segment is twice this value.

            "twist" : (float, array, or string, optional)
                Gives the geometric twist of the wing. If specified as a float, then this is simply the
                mounting angle of the wing segment and the segment will have no further twist. If specified
                as an array, the array gives the twist as a function of span. The first column gives the
                span location as a fraction of the total span. The second column gives the twist at that
                span location. If specified as a string, this string must contain the path to a csv file
                containing the twist data formatted in columns, as with the array. For properties as a
                function of span, MachUp will linearly interpolate intermediate values. Defaults to 0.

            "dihedral" : (float, array, or string, optional)
                Gives the dihedral of the wing segment. Defined the same as "twist". Defaults to 0.

            "sweep" : (float, array, or string, optional)
                Gives the sweep angle of the wing segment. Defined the same as "twist". Defaults to 0.

            "chord" : (float, array, or string, optional)
                Gives the chord length of the wing segment. Defined the same as "twist". Defaults to 1.

            "airfoil" : (string or array, optional)
                Gives the section airfoil(s) of the wing segment. Can be the name of any airfoil defined
                under "airfoils" in this object. If specified as an array, the array gives the airfoil
                as a function of span. The first column gives the span location, as with "twist", and the
                second column gives the name of the airfoil at that location. Can also be the path to a
                csv file containing the airfoil distribution formatted in columns, as with the array.
                Defaults to the name of the first airfoil listed under "airfoils".

            "grid" : (uint, optional)
                Number of horseshoe vortices used to model the wing segment in the numerical lifting-line
                algorithm. Defaults to 40.

            "use_clustering" : (bool, optional)
                If 1, control points will be distributed using cosine clusering. Otherwise, points will
                be distributed linearly. Defaults to 1.

            "control_surface" : (dict, optional)
                Defines a control surface on the trailing edge of the wing segment.

                "root_span" : (float, optional)
                    The span location, as a fraction of total span, where the control surface begins.
                    Defaults to 0.2.

                "tip_span" : (float, optional)
                    The span location, as a fraction of total span, where the control surface ends.
                    Defaults to 0.8.

                "chord_fraction" : (float, array, or string, optional)
                    The depth of the control surface, as a fraction of the local chord length. Defined
                    the same as "twist". If an array or file is specified, however, the start and end 
                    of the data must coincide with "root_span" and "tip_span", respectively. Defaults
                    to 0.25.

                "control_mixing" : (dict)
                    Determines which control inputs move this control surface. A control surface can be
                    affected by any number of controls.

                    "<CONTROL_NAME>" : (dict)
                        Corresponds to one of the controls listed in "controls".

                        "symmetric" : (bool)
                            Specifies whether this control causes the control surface to deflect
                            symmetrically (e.g. when using elevons, the elevator control should
                            deflect the control surfaces symmetrically, whereas the aileron control
                            should not).

                        "mix" : (float)
                            Linearly maps the control deflection to the control surface deflection. The
                            control deflection will be multiplied by this value and then applied to the
                            control surface.

### Airfoil Data File
An airfoil in MachUp can be defined as each coefficient being a function of angle of attack and Reynolds
number. This is done by specifying a path to a csv file describing the airfoil coefficients.