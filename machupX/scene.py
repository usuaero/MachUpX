from .helpers import _check_filepath,_convert_units,_vectorized_convert_units
from .airplane import Airplane

import json
from skaero.atmosphere import coesa
import numpy as np
import scipy.interpolate as sinterp

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

        # Store unit system
        self._unit_sys = self._input_dict.get("units","English")

        # Initialize aircraft geometries
        for airplane_name in self._input_dict["scene"]["aircraft"]:
            airplane_file = self._input_dict["scene"]["aircraft"][airplane_name]["file"]
            state = self._input_dict["scene"]["aircraft"][airplane_name].get("state",{})
            control_state = self._input_dict["scene"]["aircraft"][airplane_name].get("control_state",{})

            self.airplanes[airplane_name] = Airplane(airplane_name, airplane_file, state=state, control_state=control_state)

        # Setup atmospheric property getter functions
        self._get_density = self._initialize_density_getter(self._input_dict["scene"].get("atmosphere", {}).get("rho", None))
        self._get_wind = self._initialize_wind_getter(self._input_dict["scene"].get("atmosphere", {}).get("V_wind", None))


    def _initialize_density_getter(self, rho):

        # No parameter specified by user
        if rho is None:
            self._constant_rho = self._density_from_atmos_table(0.0, "standard")
            def density_getter(position):
                return self._constant_rho

        # Constant value
        elif isinstance(rho, float):
            self._constant_rho = rho
            def density_getter(position):
                return self._constant_rho

        # Constant value with units
        elif isinstance(rho[0], float) and isinstance(rho[1], str):
            self._constant_rho = _convert_units(rho[0],rho[1],self._unit_sys)
            def density_getter(position):
                return self._constant_rho

        # Density array given in file
        elif isinstance(rho, str) and ".csv" in rho:
            _check_filepath(rho, ".csv")
            with open(rho, 'r') as density_file:
                density_array = np.genfromtxt(density_file, delimiter=',', dtype=None, encoding='utf-8')

        # Array directly specified in JSON object
        elif any(isinstance(row, list) for row in rho):
            density_array = np.asarray(rho) # This is simply stored for now. It is processed below.

        # Atmospheric table name
        elif isinstance(rho, str):
            # Profile
            if not rho in ["standard"]:
                raise IOError("{0} is not an allowable profile name.".format(rho))

            def density_getter(position):
                return self._density_from_atmos_table(position[2], rho)

        # Improper specification
        else:
            raise IOError("Density definition {0} could not be parsed.".format(rho))

        # Handle interpolation of arrays
        try:
            # Convert units
            if isinstance(density_array[-1,0], str): # A unit specification has been given
                units = density_array[-1,:]
                density_data = density_array[:-1,:].astype(np.float)
                self._density_data = _vectorized_convert_units(density_data, units, self._unit_sys)

            else: # No units given
                self._density_data = density_array[:,:].astype(np.float)

            # Create getters
            if self._density_data.shape[1] is 2: # Density profile

                def density_getter(position):
                    return np.interp(position[2], self._density_data[:,0], self._density_data[:,1])

            elif self._density_data.shape[1] is 4: # Density field
                self._density_field_interpolator = sinterp.LinearNDInterpolator(self._density_data[:,:3],self._density_data[:,3])

                def density_getter(position):
                    return self._density_field_interpolator(position).item()

            else:
                raise IOError("Density array {0} has the wrong number of columns.".format(rho))

        except NameError:
            pass # density_array is not defined and we can simply move on

        return density_getter
        

    def _density_from_atmos_table(self, altitude, profile):
        # Computes the density at a given altitude

        if self._unit_sys == "English":
            alt = _convert_units(altitude, "ft", "SI")
        else: 
            alt = altitude

        if profile == "standard":
            _,_,_,rho = coesa.table(alt)

        if self._unit_sys == "English":
            rho = _convert_units(rho, "kg/m^3", "English")

        return rho

    
    def _initialize_wind_getter(self, V_wind):

        # No parameter specified by user
        if V_wind is None:
            def wind_getter(position):
                return np.asarray([0,0,0])

        # Constant value
        elif isinstance(V_wind, list):
            if len(V_wind) == 4 and isinstance(V_wind[3], str): # Units specified
                self._constant_wind = _vectorized_convert_units(V_wind[:3],V_wind[3],self._unit_sys)
            else: # No units specified
                self._constant_wind = np.asarray(V_wind)
            def wind_getter(position):
                return self._constant_wind

        # Wind array given in file
        elif isinstance(V_wind, str) and ".csv" in V_wind:
            _check_filepath(V_wind, ".csv")
            with open(V_wind, 'r') as wind_file:
                wind_array = np.genfromtxt(wind_file, delimiter=',', dtype=None, encoding='utf-8')

        # Array directly specified in JSON object
        elif any(isinstance(row, list) for row in rho):
            wind_array = np.asarray(V_wind) # This is simply stored for now. It is processed below.

        # Improper specification
        else:
            raise IOError("Wind velocity definition {0} could not be parsed.".format(V_wind))

        # Setup array interpolater
        try:

            # Convert units
            if isinstance(velocity_array[-1,0], str): # A unit specification has been given
                units = velocity_array[-1,:]
                velocity_data = velocity_array[:-1,:].astype(np.float)
                self._velocity_data = _vectorized_convert_units(velocity_data, units, self._unit_sys)

            else: # No units given
                self._velocity_data = velocity_array[:,:].astype(np.float)

            # Create getters
            if self._velocity_data.shape[1] is 6: # Velocity field
                self._velocity_field_x_interpolator = sinterp.LinearNDInterpolator(self._velocity_data[:,:3],self._velocity_data[:,3])
                self._velocity_field_y_interpolator = sinterp.LinearNDInterpolator(self._velocity_data[:,:3],self._velocity_data[:,4])
                self._velocity_field_z_interpolator = sinterp.LinearNDInterpolator(self._velocity_data[:,:3],self._velocity_data[:,5])

                def wind_getter(position):
                    Vx = self._velocity_field_x_interpolator(position)
                    Vy = self._velocity_field_y_interpolator(position)
                    Vz = self._velocity_field_z_interpolator(position)
                    return np.asarray([Vx, Vy, Vz])

            elif self._velocity_data.shape[1] is 4: # Velocity profile

                def wind_getter(position):
                    Vx =  np.interp(position[2], self._velocity_data[:,0], self._density_data[:,1])
                    Vy =  np.interp(position[2], self._velocity_data[:,0], self._density_data[:,2])
                    Vz =  np.interp(position[2], self._velocity_data[:,0], self._density_data[:,3])
                    return np.asarray([Vx, Vy, Vz])

            else:
                raise IOError("Velocity array {0} has the wrong number of columns.".format(V_wind))

        except NameError:
            pass # wind_array isn't defined and we simply move on

        return wind_getter