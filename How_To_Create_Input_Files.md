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

Boolean values are defined with a 1 or a 0.

## Units

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

For array inputs, such as a density profile or chord distribution, the units are appended as another row
in the array:

```python
"rho" : [[0.0,1.225],
         [2000.0,1.0066],
         [4000.0,0.81935],
         ["m","kg/m^3"]]
```

The following measurements can be defined with the accompanying units:

Position/displacement/length: "ft", "m", "in", "cm"
Velocity: "ft/s", "m/s", "mph", "kph", "kn"
Angular deflection/position: "deg", "rad"
Angular rate: "deg/s", "rad/s"
Density: "slug/ft^3", "kg/m^3"
Weight: "lbf", "N"

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
            
            "V_wind" : (vector or "file")
                If a 1D array is given, this is assumed to be the wind velocity vector given in
                flat-earth coordinates which is constant throughout the scene. If specified as 
                "file", MachUp will pull the local velocity from the filename given by "path"
                (see below). NOTE: This value is optional if the state type of all aircraft is
                specified as "aerodynamic".

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

                    "velocity" : (vector)
                        Velocity of the aircraft in flat-earth coordinates.

                    "orientation" : (vector)
                        Orientation of the aircraft in flat-earth coordinates. If this is a 3-element 
                        vector it is assumed an Euler angle formulation is used. If this is a 4-element 
                        vector it is assumed a quaternion formulation is used.

                    "rates" : (vector)
                        Angular rate of the aircraft in flat-earth coordinates.

                    If "type" is specified as "aerodynamic", the following keys must be specified:

                    "position" : (vector)
                        Position of the origin of the aircraft's body-fixed coordinate system in flat-earth
                        coordinates.

                    "rates" : (vector)
                        Angular rate of the aircraft in flat-earth coordinates.

                    "V_mag" : (float)
                        Magnitude of the local wind vector.

                    "alpha" : (float)
                        Aerodynamic angle of attack.

                    "beta" : (float)
                        Aerodynamic sideslip angle.

                "control_state" : (dict, optional)
                    Describes the control deflections. The number and names of controls are arbitrary and 
                    may be specified by the user. This is discussed more in depth as part of the aircraft 
                    object. If the aircraft has controls but no state is specified, all deflections will 
                    be assumed to be zero.

                    "<CONTROL_NAME>" : (dict)

                        "deflection" : (float)
                            Control surface deflection.

## Aircraft Object
Describes an aircraft.

    "name" : (string)
        The name of the aircraft.

    "CG" : (1D array)
        Location of the aircraft's center of gravity in body-fixed coordinates. For simplicity, the CG is
        often assumed to be coincident with the body-fixed origin.

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
        Defines the airfoil section parameters for all airfoils used on the aircraft. A dict defining an 
        airfoil has the following structure:

        "<AIRFOIL_NAME>" : (dict)

            "type" : (string)
                The type of information describing the airfoil. If "linear", the following keys are 
                required:

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
                Path to file containing either a JSON object describing the airfoil formatted as above or 
                tabulated data of airfoil coefficients as a function of angle of attack and Reynolds 
                number (described below).

        Any number of airfoils can be defined for the aircraft simply by repeating the above structure
        within the "airfoils" dict. MachUp pulls from these airfoil definitions as needed, depending on 
        which airfoils are specified for the wings.

    "wing_segments" : (dict)
        Gives the lifting surfaces for the aircraft. Wings, stabilizers, fins, etc. are all treated the 
        same in numerical lifting-line and so should be included here as wings. MachUp is set up so the
        user can define complex geometries by attaching the ends of different wing segments together (for 
        an example, see the /examples directory). Any number of wing segments can be defined by the user.

        "<WING_SEGMENT_NAME>" : (dict)

            "name" : (string)

            "ID" : (uint)
                ID tag of the wing segments used for specifying which other wing segments are defined 
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

                "ID" : (uint)
                    ID of the wing segment this wing segment's origin is being defined relative to. If 0,
                    this wing segment's origin will be defined relative to the aircraft's body-fixed 
                    origin.

                "location" : (string)
                    May be "root" or "tip". Defines whether this wing segment's origin should be defined 
                    relative to the root or tip of the other wing segment.

                "dx" : (float)
                    Displacement of the origin from the selected reference point in the body-fixed x-
                    direction.

                "dy" : (float)
                    Displacement of the origin from the selected reference point in the body-fixed y-
                    direction. NOTE: If "side" is specified as "both", changing this value will shift
                    both sides of the wing segment in the SAME direction. The effect is not mirrored.

                "dz" : (float)
                    Displacement of the origin from the selected reference point in the body-fixed z-
                    direction.

                "y_offset" : (float)
                    Distance the origin should be shifted from the centerline (positive offset 
                    corresponds to outward). If "side" is specified as "both", this effect is mirrored.
            
            "span" : (float)
                Length of the wing segment, discounting sweep. If "side" is specified as "both", the total
                span of the segment is twice this value.

            "twist" : (float, array, or string)
                Gives the geometric twist of the wing. If specified as a float, then this is simply the
                mounting angle of the wing segment and the segment will have no further twist. If specified
                as an array, the array gives the twist as a function of span. The first column gives the
                span location as a fraction of the total span. The second column gives the twist at that
                span location. If specified as a string, this string must contain the path to a csv file
                containing the twist data formatted in columns, as with the array. For properties as a
                function of span, MachUp will linearly interpolate intermediate values.

            "dihedral" : (float, array, or string)
                Gives the dihedral of the wing segment. Defined the same as "twist".

            "sweep" : (float, array, or string)
                Gives the sweep angle of the wing segment. Defined the same as "twist".

            "chord" : (float, array, or string)
                Gives the chord length of the wing segment. Defined the same as "twist".

            "airfoil" : (string or array)
                Gives the section airfoil(s) of the wing segment. Can be the name of any airfoil defined
                under "airfoils" in this object. If specified as an array, the array gives the airfoil
                as a function of span. The first column gives the span location, as with "twist", and the
                second column gives the name of the airfoil at that location. Can also be the path to a
                csv file containing the airfoil distribution formatted in columns, as with the array.

            "grid" : (uint)
                Number of horseshoe vortices used to model the wing segment in the numerical lifting-line
                algorithm.

            "clustering" : (bool, optional)
                If 1, control points will be distributed using cosine clusering. Otherwise, points will
                be distributed linearly. Defaults to 1.

            "control_surface" : (dict, optional)
                Defines a control surface on the wing segment.

                "root_span" : (float)
                    The span location, as a fraction of total span, where the control surface begins.

                "tip_span" : (float)
                    The span location, as a fraction of total span, where the control surface ends.

                "chord_fraction" : (float, array, or string)
                    The depth of the control surface, as a fraction of the local chord length. Defined
                    the same as "twist". If an array or file is specified, however, the start and end 
                    of the data must correspond with "root_span" and "tip_span", respectively.

                "control_mixing" : (dict)
                    Determines which control inputs move this control surface. A control surface can be
                    affected by any numer of controls.

                    "<CONTROL_NAME>" : (dict)
                        Corresponds to one of the controls listed in "controls".

                        "symmetric" : (bool)
                            Specifies whether this control causes the control surface to deflect
                            symmetrically (e.g. when using elevons, the elevator control should
                            deflect the control surfaces symmetrically, whereas the aileron control
                            should not).

                        "mix" : (float)
                            Linearly maps the control deflection to the control surface deflection.