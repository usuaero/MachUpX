from .helpers import _check_filepath, _import_value

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

    def __init__(self, name, filename, unit_system, state={}, control_state={}):

        self.name = name
        self._unit_sys = unit_system

        self._CG = None
        self.W = None
        self.S_w = None
        self.l_ref_lon = None
        self.l_ref_lat = None
        self._control_names = None
        self._wings = {}

        self._load_params(filename)
        self._initialize_state(state)
        self._initialize_controls(control_state)

    def _load_params(self, filename):
        # Load JSON object
        _check_filepath(filename,".json")
        with open(filename) as json_handle:
            self._input_dict = json.load(json_handle)

        # Set airplane global params
        self._CG = _import_value("CG", self._input_dict, self._unit_sys, [0,0,0])

    def _initialize_state(self, state):
        # Sets the state vector from the provided dictionary
        pass

    def _initialize_controls(self, control_state):
        # Sets the control vector from the provided dictionary
        pass