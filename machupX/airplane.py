from .helpers import _check_filepath, _import_value, _quaternion_transform, _quaternion_inverse_transform, _euler_to_quaternion, _quaternion_to_euler
from .wing_segment import WingSegment
from .airfoil import Airfoil

import json
import numpy as np

class Airplane:
    """A class defining an airplane.

    Parameters
    ----------
    name : string
        Name of the airplane.

    airplane_input : string or dict
        Path to the JSON object describing the airplane or dictionary
        containing the same information.

    state : dict
        Dictionary describing the initial state vector of the airplane.

    control_state : dict
        Dictionary describing the initial state of the airplane's controls.

    Returns
    -------
    Airplane
        Returns a newly create airplane object.

    Raises
    ------
    IOError
        If the input filepath or filename is invalid.
    """

    def __init__(self, name, airplane_input, unit_system, init_state={}, init_control_state={}):

        self.name = name
        self._unit_sys = unit_system
        
        self.wing_segments = {}
        self._airfoil_database = {}
        self._N = 0

        self._load_params(airplane_input)
        self.set_state(init_state)
        self._create_airfoil_database()
        self._create_origin_segment()
        self._load_wing_segments()
        self._check_reference_params()
        self._initialize_controls(init_control_state)


    def _load_params(self, airplane_input):
        if isinstance(airplane_input, str):
            # Load JSON object
            _check_filepath(airplane_input, ".json")
            with open(airplane_input) as json_handle:
                self._input_dict = json.load(json_handle)
        elif isinstance(airplane_input, dict):
            self._input_dict = airplane_input
        else:
            raise IOError("{0} is not an allowed airplane definition. Must be path or dictionary.".format(airplane_input))

        # Set airplane global params
        self.CG = _import_value("CG", self._input_dict, self._unit_sys, [0,0,0])
        self.W = _import_value("weight", self._input_dict, self._unit_sys, None)
        self.S_w = _import_value("area", self._input_dict.get("reference", {}), self._unit_sys, -1)
        self.l_ref_lon = _import_value("longitudinal_length", self._input_dict.get("reference", {}), self._unit_sys, -1)
        self.l_ref_lat = _import_value("lateral_length", self._input_dict.get("reference", {}), self._unit_sys, -1)
        self._control_names = self._input_dict.get("controls", [])


    def set_state(self, state, v_wind=np.array([0, 0, 0])):
        """Sets the state of the aircraft from the provided dictionary.
        Parameters
        ----------
        state : dict
            A dictionary describing the state of the aircraft. Same specification 
            as given in How_To_Create_Input_Files.

        v_wind : ndarray
            The local wind vector at the aircraft body-fixed origin in flat-earth 
            coordinates. Defaults to [0.0, 0.0, 0.0].
        """

        self.state_type = _import_value("type", state, self._unit_sys, None)
        self.p_bar = _import_value("position", state, self._unit_sys, [0, 0, 0])
        self.w = _import_value("angular_rates", state, self._unit_sys, [0, 0, 0])

        # Set up orientation quaternion
        self.q = _import_value("orientation", state, self._unit_sys, [1.0, 0.0, 0.0, 0.0]) # Default aligns the aircraft with the flat-earth coordinates
        if self.q.shape[0] == 3: # Euler angles
            self.q = _euler_to_quaternion(self.q)

        elif self.q.shape[0] == 4: # Quaternion
            # Check magnitude
            if abs(np.linalg.norm(self.q)-1.0) > 1e-10:
                raise IOError("Magnitude of orientation quaternion must be 1.0.")

        else:
            raise IOError("{0} is not an allowable orientation definition.".format(self.q))

        # Rigid-body definition
        if self.state_type == "rigid-body":
            if "alpha" in list(state.keys()) or "beta" in list(state.keys()):
                raise IOError("Alpha and beta are not allowed as part of a rigid-body state specification.")

            # Set up velocity
            self.v = _import_value("velocity", state, self._unit_sys, None) # The user has specified translational velocity in flat-earth coordinates, so wind doesn't matter
            if not isinstance(self.v, np.ndarray):
                raise IOError("For a rigid-body state definition, 'velocity' must be a 3-element vector.")

        # Aerodynamic definition
        elif self.state_type == "aerodynamic":

            # Set up velocity
            v_value = _import_value("velocity", state, self._unit_sys, None)
            if isinstance(v_value, float): # User has given a velocity magnitude, so alpha and beta are also needed
                alpha = _import_value("alpha", state, self._unit_sys, 0.0)
                beta = _import_value("beta", state, self._unit_sys, 0.0)

                C_a = np.cos(np.radians(alpha))
                S_a = np.sin(np.radians(alpha))
                C_B = np.cos(np.radians(beta))
                S_B = np.sin(np.radians(beta))

                v = np.zeros(3)
                denom = np.sqrt(1-S_a**2*S_B**2)
                # This formulation gives the components of aircraft velocity in body-fixed coordinates (Mech of Flight Eqs. 7.1.10-12)
                v[0] = v_value*C_a*C_B/denom
                v[1] = v_value*C_a*S_B/denom
                v[2] = v_value*S_a*C_B/denom

                self.v = _quaternion_inverse_transform(self.q, v)+v_wind # Transform to flat-earth coordinates and account for wind (Mech of Flight Eq. 7.1.1)

            elif isinstance(v_value, np.ndarray) and v_value.shape[0] == 3: # User has given u, v, and w
                if "alpha" in list(state.keys()) or "beta" in list(state.keys()):
                    raise IOError("Alpha and beta may not be specified along with a freestream vector.")

                # The negative of u, v, and w gives the aircraft's body-fixed velocity relative to the wind
                self.v = _quaternion_inverse_transform(self.q, -v_value)+v_wind

            else:
                raise IOError("{0} is not an allowable velocity definition.", v_value)

        else:
            raise IOError("{0} is not an acceptable state type.".format(state_type))


    def get_alpha(self, v_wind=np.array([0, 0, 0])):
        """Returns the aircraft's angle of attack.

        Parameters
        ----------
        v_wind : ndarray
            The local wind vector at the aircraft body-fixed origin in flat-earth 
            coordinates. Defaults to [0.0, 0.0, 0.0].
        """
        pass


    def get_beta(self, v_wind=np.array([0, 0, 0])):
        """Returns the aircraft's sideslip angle.

        Parameters
        ----------
        v_wind : ndarray
            The local wind vector at the aircraft body-fixed origin in flat-earth 
            coordinates. Defaults to [0.0, 0.0, 0.0].
        """
        pass


    def _initialize_controls(self, init_control_state):
        # Initializes the control surfaces on the airplane
        self._control_symmetry = {}
        controls = self._input_dict.get("controls", {})
        for key in controls:
            self._control_symmetry[key] = controls[key]["is_symmetric"]

        self.set_control_state(control_state=init_control_state)
        

    def _create_origin_segment(self):
        # Create a wing segment which has no properties but which other segments 
        # connect to.
        origin_dict = {
            "ID" : 0,
            "is_main" : False
        }
        self._origin_segment = WingSegment("origin", origin_dict, "both", self._unit_sys, self._airfoil_database)

    
    def add_wing_segment(self, wing_segment_name, input_dict):
        """Adds a wing segment to the airplane.

        Parameters
        ----------
        wing_segment_name : str
            Name of the wing segment.

        input_dict : dict
            Dictionary describing the wing segment. Same as specified for input files.

        Returns
        -------

        Raises
        ------
        IOError
            If the input is improperly specified.
        """

        #Let me take a moment to explain the structure of wing segments in MachUpX. This is
        #for the sake of other developers. The way we have decided to define wing segements 
        #makes them fall very naturally into a tree-type structure. Any given wing segment 
        #is attached (we use this term loosely; more accurately, the position of one wing 
        #segment is defined relative to another) to another wing segment or the origin. 
        #Eventually, these all lead back to the origin. The origin here is a "dummy" wing 
        #segment which has no other properties than an ID of 0. Adding a wing segment is done
        #recursively via the tree. Each wing segment knows which wing segments attach to it.
        #However, no wing segment knows who it attaches to, only the location of its origin. 

        #The tree structure makes certain operations, such as integrating forces and moments 
        #and applying structural deformations, very natural. However, generating the lifting-
        #line matrix equations from this structure is very cumbersome. Therefore, we also 
        #store references to each wing segment at the Airplane level in a list. This makes 
        #generating the lifting-line matrix much more friendly. This makes the code a little 
        #more fragile, but this is Python and we assume the user is being responsible.
        
        if wing_segment_name in self.wing_segments.keys():
            raise IOError("Wing segment {0} already exists in this airplane.".format(wing_segment_name))

        side = input_dict.get("side")
        if not (side == "left" or side == "right" or side == "both"):
            raise IOError("{0} is not a proper side designation.".format(side))

        if side == "left" or side == "both":
            self.wing_segments[wing_segment_name+"_left"] = self._origin_segment.attach_wing_segment(wing_segment_name+"_left", input_dict, "left", self._unit_sys, self._airfoil_database)
            self._N += self.wing_segments[wing_segment_name+"_left"]._N

        if side == "right" or side == "both":
            self.wing_segments[wing_segment_name+"_right"] = self._origin_segment.attach_wing_segment(wing_segment_name+"_right", input_dict, "right", self._unit_sys, self._airfoil_database)
            self._N += self.wing_segments[wing_segment_name+"_right"]._N


    def _load_wing_segments(self):
        # Reads in the wing segments from the input dict and attaches them
        for key in self._input_dict.get("wings", {}):
            self.add_wing_segment(key, self._input_dict["wings"][key])


    def _check_reference_params(self):
        # If the reference area and lengths have not been set, this takes care of that.

        # Reference area
        if self.S_w == -1:
            self.S_w = 0.0
            for (_, wing_segment) in self.wing_segments.items():
                if wing_segment.is_main:
                    self.S_w += np.sum(wing_segment.dS)

        # Lateral reference length
        if self.l_ref_lat == -1:
            self.l_ref_lat = 0.0
            for (_, wing_segment) in self.wing_segments.items():
                if wing_segment.is_main and wing_segment._side == "right":
                    self.l_ref_lat += wing_segment.b

        # Longitudinal reference length
        if self.l_ref_lon == -1:
            self.l_ref_lon = self.S_w/(2*self.l_ref_lat)


    def delete_wing_segment(self, wing_segment_name):
        """Removes the specified wing segment from the airplane. Removes both sides.

        Parameters
        ----------
        wing_segment_name : str
            Name of the wing segment.

        Returns
        -------

        Raises
        ------
        ValueError
            If the wing segment does not exist

        RuntimeError
            If the wing segment has other segments attached to it.
        """
        #TODO: Do this
        pass


    def _get_wing_segment(self, wing_segment_name):
        # Returns a reference to the specified wing segment. Use with caution. Or just don't use.
        return self._origin_segment._get_attached_wing_segment(wing_segment_name)

    
    def _create_airfoil_database(self):
        # Creates a dictionary of all the airfoils. This dictionary is then passed to each 
        # wing segment when it gets created for the wing segment to use.

        airfoils = self._input_dict.get("airfoils", {"default" : {} })

        # Load airfoil database from separate file
        if isinstance(airfoils, str):
            _check_filepath(airfoils, ".json")
            with open(airfoils, 'r') as airfoil_db_handle:
                airfoil_dict = json.load(airfoil_db_handle)

        # Load from airplane dict
        elif isinstance(airfoils, dict):
            airfoil_dict = airfoils

        else:
            raise IOError("'airfoils' must be a string or dict.")

        for key in airfoil_dict:
            self._airfoil_database[key] = Airfoil(key, airfoil_dict[key])


    def get_num_cps(self):
        """Returns the total number of control points on the aircraft.

        Returns
        -------
        int
            Number of control points on the aircraft.
        """
        return self._N


    def get_v_inf(self):
        """Returns the freestream velocity acting on the airplane due to its translation.
        Note this does not account for wind. Wind must be handled at the Scene level.

        Returns
        -------
        ndarray
            Vector of freestream velocity.
        """
        return -_quaternion_transform(self.q, self.v)


    def set_control_state(self, control_state={}):
        """Sets the control surface deflections on the airplane using the control mapping.

        Parameters
        ----------
        control_state : dict
            A set of key-value pairs where the key is the name of the control and the 
            value is the deflection. For positive mapping values, a positive deflection 
            here will cause a downward deflection of symmetric control surfaces and 
            downward deflection of the right surface for anti-symmetric control surfaces.
            Units may be specified as in the input file. Any deflections not given will 
            default to zero.
        """
        for _,wing_segment in self.wing_segments.items():
            wing_segment.apply_control(control_state, self._control_symmetry)
