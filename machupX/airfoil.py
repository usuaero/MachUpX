from .helpers import _check_filepath, _vectorized_convert_units, _import_value
from .poly_fits import multivariablePolynomialFit, multivariablePolynomialFunction

import numpy as np
import json

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
        self._define_vectorized_getters()

    
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
        pass


    def _define_vectorized_getters(self):
        # Creates vectorized functions to return CL, CD, CL,a, Cm, aL0, etc.
        # For each of these, the only parameter is inputs. This is a vector
        # of parameters which can affect the airfoil coefficients. The first
        # three are always alpha, Reynolds number, and Mach number, in that
        # order.

        # Lift coefficient getter
        def CL(inputs):
            if self._type == "linear":
                CL = self._CLa*(inputs-self._aL0)
                if CL > self._CL_max or CL < -self._CL_max:
                    CL = np.sign(CL)*self._CL_max
                return CL

        self.get_CL = np.vectorize(CL)
        
        # Drag coefficient getter
        def CD(inputs):
            if self._type == "linear":
                CL = self.get_CL(inputs)
                return self._CD0+self._CD1*CL+self._CD2*CL**2

        self.get_CD = np.vectorize(CD)

        # Moment coefficient getter
        def Cm(inputs):
            if self._type == "linear":
                return self._Cma*inputs+self._CmL0

        self.get_Cm = np.vectorize(Cm)

        # Lift slope getter
        def CLa(inputs=None):
            if self._type == "linear":
                return self._CLa

        self.get_CLa = np.vectorize(CLa)

        # Zero-lift angle of attack getter
        def aL0(inputs=None):
            if self._type == "linear":
                return self._aL0

        self.get_aL0 = np.vectorize(aL0)

        # Derivative of lift coef wrt Mach number
        def CLM(inputs=None):
            if self._type == "linear":
                return self._CLM

        self.get_CLM = np.vectorize(CLM)

        # Derivative of lift coef wrt Re
        def CLRe(inputs=None):
            if self._type == "linear":
                return self._CLRe