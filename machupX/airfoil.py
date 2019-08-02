from .helpers import _check_filepath, _vectorized_convert_units, _import_value
from .poly_fits import multivariablePolynomialFit, multivariablePolynomialFunction

import numpy as np
import json
import copy

class Airfoil:
    """A class defining an airfoil.

    Parameters
    ----------
    name : str
        Name of the airfoil.

    input_dict : dict
        Dictionary describing the airfoil.

    Returns
    -------
    Airfoil
        A newly created airfoil object.

    Raises
    ------
    IOError
        If the input is invalid.
    """


    def __init__(self, name, input_dict):

        self.name = name
        self._input_dict = input_dict
        self._type = _import_value("type", self._input_dict, "SI", None) # Unit system doesn't matter for these

        self._initialize_data()

    
    def _initialize_data(self):
        # Initializes the necessary data structures for the airfoil
        if self._input_dict.get("generate_database", False):
            self._generate_database()

        elif self._type == "linear":

            # Load from file
            try:
                filename = self._input_dict["path"]
                _check_filepath(filename, ".json")
                with open(filename, 'r') as airfoil_file_handle:
                    params = json.load(airfoil_file_handle)

            # Load from input dict
            except KeyError:
                params = self._input_dict

            # Save params
            self._aL0 = _import_value("aL0", params, "SI", 0.0) # Again, the unit system doesn't matter
            self._CLa = _import_value("CLa", params, "SI", 2*np.pi)
            self._CmL0 = _import_value("CmL0", params, "SI", 0.0)
            self._Cma = _import_value("Cma", params, "SI", 0.0)
            self._CD0 = _import_value("CD0", params, "SI", 0.0)
            self._CD1 = _import_value("CD1", params, "SI", 0.0)
            self._CD2 = _import_value("CD2", params, "SI", 0.0)
            self._CL_max = _import_value("CL_max", params, "SI", np.inf)

            self._CLM = _import_value("CLM", params, "SI", 0.0)
            self._CLRe = _import_value("CLRe", params, "SI", 0.0)

        elif self._type == "nonlinear":

            # Load coefficient data
            try:
                filename = self._input_dict["path"]
                _check_filepath(filename, ".csv")
                with open(filename, 'r') as airfoil_file_handle:
                    self._nonlinear_data = np.genfromtxt(airfoil_file_handle, dtype=None)
            except KeyError:
                raise IOError("'path' must be specified for nonlinear airfoil {0}".format(self.name))

        else:
            raise IOError("'{0}' is not an allowable airfoil type.".format(self._type))


    def _generate_database(self):
        # Generates a database of airfoil parameters from the section geometry
        #TODO: Implement this
        raise IOError("Generateing an airfoil database is not yet allowed in this version of MachUpX.")


    def get_CL(self, inputs):
        """Returns the coefficient of lift.

        Parameters
        ----------
        inputs : ndarray
            Parameters which can affect the airfoil coefficients. The first
            three are always alpha, Reynolds number, and Mach number. Fourth 
            is flap efficiency and fifth is flap deflection.

        Returns
        -------
        float
            Lift coefficient
        """
        if self._type == "linear":
            CL = self._CLa*(inputs[0]-self._aL0+inputs[3]*inputs[4])
            if CL > self._CL_max or CL < -self._CL_max:
                CL = np.sign(CL)*self._CL_max
            return CL


    def get_CD(self, inputs):
        """Returns the coefficient of drag

        Parameters
        ----------
        inputs : ndarray
            Parameters which can affect the airfoil coefficients. The first
            three are always alpha, Reynolds number, and Mach number. Fourth 
            is flap efficiency and fifth is flap deflection.

        Returns
        -------
        float
            Drag coefficient
        """
        if self._type == "linear":
            delta_flap = inputs[4]
            inputs_wo_flap = copy.copy(inputs)
            inputs_wo_flap[3:] = 0.0
            CL = self.get_CL(inputs_wo_flap)
            CD_flap = 0.002*np.abs(delta_flap)*180/np.pi # A rough estimate for flaps
            return self._CD0+self._CD1*CL+self._CD2*CL**2+CD_flap


    def get_Cm(self, inputs):
        """Returns the moment coefficient

        Parameters
        ----------
        inputs : ndarray
            Parameters which can affect the airfoil coefficients. The first
            three are always alpha, Reynolds number, and Mach number. Fourth 
            is flap efficiency and fifth is flap deflection.

        Returns
        -------
        float
            Moment coefficient
        """
        if self._type == "linear":
            return self._Cma*inputs[0]+self._CmL0+inputs[3]*inputs[4]


    def get_aL0(self, inputs):
        """Returns the zero-lift angle of attack

        Parameters
        ----------
        inputs : ndarray
            Parameters which can affect the airfoil coefficients. The first
            three are always alpha, Reynolds number, and Mach number. Fourth 
            is flap efficiency and fifth is flap deflection.

        Returns
        -------
        float
            Zero-lift angle of attack
        """
        if self._type == "linear":
            return self._aL0


    def get_CLM(self, inputs):
        """Returns the lift slope with respect to Reynolds number

        Parameters
        ----------
        inputs : ndarray
            Parameters which can affect the airfoil coefficients. The first
            three are always alpha, Reynolds number, and Mach number. Fourth 
            is flap efficiency and fifth is flap deflection.

        Returns
        -------
        float
            Lift slope with respect to Reynolds number
        """
        if self._type == "linear":
            return self._CLM


    def get_CLRe(self, inputs):
        """Returns the lift slope with respect to Reynolds number

        Parameters
        ----------
        inputs : ndarray
            Parameters which can affect the airfoil coefficients. The first
            three are always alpha, Reynolds number, and Mach number. Fourth 
            is flap efficiency and fifth is flap deflection.

        Returns
        -------
        float
            Lift slope with respect to Reynolds number
        """
        if self._type == "linear":
            return self._CLRe


    def get_CLa(self, inputs):
        """Returns the lift slope

        Parameters
        ----------
        inputs : ndarray
            Parameters which can affect the airfoil coefficients. The first
            three are always alpha, Reynolds number, and Mach number. Fourth 
            is flap efficiency and fifth is flap deflection.

        Returns
        -------
        float
            Lift slope
        """
        if self._type == "linear":
            return self._CLa
