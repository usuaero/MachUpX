from .helpers import _check_filepath

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

    def __init__(self, name, filename, state={}, control_state={}):

        self.name = name
        _check_filepath(filename,".json")

        self._load_params(filename)
        self._initialize_state(state)
        self._initialize_controls(control_state)

    def _load_params(self, filename):
        # Loads the airplane geometry from the JSON file
        pass

    def _initialize_state(self, state):
        # Sets the state vector from the provided dictionary
        pass

    def _initialize_controls(self, control_state):
        # Sets the control vector from the provided dictionary
        pass