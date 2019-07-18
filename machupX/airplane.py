from .helpers import _check_filepath, _import_value
from .wing_segment import WingSegment
from .airfoil import Airfoil

import json

class Airplane:
    """A class defining an airplane.

    Parameters
    ----------
    name : string
        Name of the airplane.

    filename : string
        Path to the JSON object describing the airplane.

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

    def __init__(self, name, ID, filename, unit_system, state={}, control_state={}):

        self.name = name
        self.ID = ID
        self._unit_sys = unit_system
        
        self._wing_segments = {}
        self._airfoil_database = {}

        self._load_params(filename)
        self._initialize_state(state)
        self._initialize_controls(control_state)
        self._create_airfoil_database()
        self._create_origin_segment()
        self._load_wing_segments()


    def _load_params(self, filename):
        # Load JSON object
        _check_filepath(filename,".json")
        with open(filename) as json_handle:
            self._input_dict = json.load(json_handle)

        # Set airplane global params
        self.CG = _import_value("CG", self._input_dict, self._unit_sys, [0,0,0])
        self.W = _import_value("weight", self._input_dict, self._unit_sys, -1)
        self.S_w = _import_value("area", self._input_dict.get("reference", {}), self._unit_sys, None)
        self.l_ref_lon = _import_value("longitudinal_length", self._input_dict.get("reference", {}), self._unit_sys, None)
        self.l_ref_lat = _import_value("lateral_length", self._input_dict.get("reference", {}), self._unit_sys, None)
        self._control_names = self._input_dict.get("controls", [])


    def _initialize_state(self, state):
        # Sets the state vector from the provided dictionary
        pass


    def _initialize_controls(self, control_state):
        # Sets the control vector from the provided dictionary
        pass
        

    def _create_origin_segment(self):
        # Create a wing segment which has no properties but which other segments 
        # connect to.
        origin_dict = {
            "ID" : 0
        }
        self._origin_segment = WingSegment("origin", origin_dict, "both", self._unit_sys, self._airfoil_database)

    
    def add_wing_segment(self, wing_segment_name, input_dict):
        """Adds a wing segment to the airplane.

        Let me take a moment to explain the structure of wing segments in MachUpX. This is
        for the sake of other developers. The way we have decided to define wing segements 
        makes them fall very naturally into a tree-type structure. Any given wing segment 
        is attached (we use this term loosely; more aaccurately, the position of one wing 
        segment is defined relative to another) to another wing segment or the origin. 
        Eventually, these all lead back to the origin. The origin here is a "dummy" wing 
        segment which has no other properties than an ID of 0. Adding a wing segment is done
        recursively via the tree. Each wing segment knows which wing segments attach to it.
        However, no wing segment knows who it attaches to, only the location of its origin. 

        The tree structure makes certain operations, such as integrating forces and moments 
        and applying structural deformations, very natural. However, generating the lifting-
        line matrix equations from this structure is very cumbersome. Therefore, we also 
        store references to each wing segment at the Airplane level in a list. This makes 
        generating the lifting-line matrix much more friendly. This makes the code a little 
        more fragile, but this is Python and we assume the user is being responsible.

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
        
        if wing_segment_name in self._wing_segments.keys():
            raise IOError("Wing segment {0} already exists in this airplane.".format(wing_segment_name))

        side = input_dict.get("side")
        if not (side == "left" or side == "right" or side == "both"):
            raise IOError("{0} is not a proper side designation.".format(side))

        if side == "left" or side == "both":
            self._wing_segments[wing_segment_name+"_left"] = self._origin_segment.attach_wing_segment(wing_segment_name+"_left", input_dict, "left", self._unit_sys, self._airfoil_database)

        if side == "right" or side == "both":
            self._wing_segments[wing_segment_name+"_right"] = self._origin_segment.attach_wing_segment(wing_segment_name+"_right", input_dict, "right", self._unit_sys, self._airfoil_database)


    def _load_wing_segments(self):
        # Reads in the wing segments from the input dict and attaches them
        for key in self._input_dict.get("wings", {}):
            self.add_wing_segment(key, self._input_dict["wings"][key])


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
        # Returns a reference to the specified wing segment. ONLY FOR TESTING!
        return self._origin_segment._get_attached_wing_segment(wing_segment_name)

    
    def _create_airfoil_database(self):
        # Creates a dictionary of all the airfoils. This dictionary is then passed to each 
        # wing segment when it gets created for the wing segment to use.

        try:
            airfoils = self._input_dict["airfoils"]
        except NameError:
            raise IOError("Airfoil database must be defined.")

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