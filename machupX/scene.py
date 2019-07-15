from .helpers import _check_filepath
from .airplane import Airplane

import json

class Scene:
    """A class defining a scene containing one or more aircraft.

    Parameters
    ----------
    input_filename : string
        Path to the JSON object specifying the scene parameters. For information on creating
        input files, see examples/How_To_Create_Input_Files.

    Raises
    ------
    IOError
        If input filepath or filename is invalid

    Examples
    --------

    """

    def __init__(self, input_filename):

        self.airplanes = {}

        _check_filepath(input_filename,".json")
        self._load_params(input_filename)

    def _load_params(self, input_filename):
        # Loads JSON object and stores input parameters and aircraft
        with open(input_filename) as input_json_handle:
            self._input_dict = json.load(input_json_handle)

        # Store solver parameters
        solver_params = self._input_dict.get("solver",{})
        self._nonlinear_solver = solver_params.get("type", "linear") == "nonlinear"
        self._solver_convergence = solver_params.get("convergence",1e-10)
        self._solver_relaxation = solver_params.get("relaxation",0.9)

        # Store default units
        self._default_units = self._input_dict.get("units","English")

        # Initialize aircraft geometry
        for airplane_name in self._input_dict["scene"]["aircraft"]:
            airplane_file = self._input_dict["scene"]["aircraft"][airplane_name]["file"]
            state = self._input_dict["scene"]["aircraft"][airplane_name].get("state",{})
            control_state = self._input_dict["scene"]["aircraft"][airplane_name].get("control_state",{})

            self.airplanes[airplane_name] = Airplane(airplane_name, airplane_file, state=state, control_state=control_state)

        # Setup atmospheric property getter functions
        # Density
        rho = self._input_dict["scene"]["atmosphere"].get("rho",None)

        # No parameter specified by user
        if rho is None:
            if self._default_units is "English":
                self._default_rho = 0.0023769 # slug/ft^3
            else:
                self._default_rho = 1.225 # kg/m^3
            self._constant_rho = True

    def _get_density(self, position_vec):
        # Returns the atmospheric density as a function of position
        if self._constant_rho:
            return self._default_rho
        pass

    def _get_wind_vel(self, position_vec):
        pass