from .helpers import _check_filepath, _vectorized_convert_units, _import_value

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

    input_list : str
        List of argument names the airfoil parameters can be a function of. The 
        first element of this list must always be "alpha" (i.e. angle of 
        attack). An arbitrary number of the following arguments be specified 
        (must be in this order):

        "Rey" : Reynolds number NOT IMPLEMENTED
        "M" : Mach number NOT IMPLEMENTED

    Returns
    -------
    Airfoil
        A newly created airfoil object.

    Raises
    ------
    IOError
        If the input is invalid.
    """

    def __init__(self, name, input_dict, input_list=["alpha"]):

        self.name = name
        self._input_dict = input_dict
        self._type = _import_value("type", self._input_dict, "SI", -1) # Unit system doesn't matter for these

        #TODO: Implement mapping of args
        if input_list[0] != "alpha":
            raise IOError("Don't do that!")

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
            self._aL0 = _import_value("aL0", params, "SI", -1) # Again, the unit system doesn't matter
            self._CLa = _import_value("CLa", params, "SI", -1)
            self._CmL0 = _import_value("CmL0", params, "SI", -1)
            self._Cma = _import_value("Cma", params, "SI", -1)
            self._CD0 = _import_value("CD0", params, "SI", -1)
            self._CD1 = _import_value("CD1", params, "SI", -1)
            self._CD2 = _import_value("CD2", params, "SI", -1)
            self._CL_max = _import_value("CL_max", params, "SI", -1)

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
        # Creates vecotrized functions to return CL, CD, and Cm

        def CL(self, inputs):
            if self._type == "linear":
                CL = self._CLa*(inputs[0]-self._aL0)
                if CL > self._CL_max or CL < -self._CL_max:
                    CL = np.sign(CL)*self._CL_max
                return CL

        CL_docstring = """Returns the coefficient of lift as a function of args.

        Parameters
        ----------
        inputs : ndarray
            Arbitrary airfoil parameters. The first is always angle of attack in radians.

        Returns
        -------
        float
            Lift coefficient.
        """

        self.get_CL = np.vectorize(CL, doc=CL_docstring, excluded={0})
        
        def CD(self, inputs):
            if self._type == "linear":
                CL = self.get_CL(*args)
                return self._CD0+self._CD1*CL+self._CD2*CL**2

        CD_docstring = """Returns the coefficient of drag as a function of args.

        Parameters
        ----------
        inputs : ndarray
            Arbitrary airfoil parameters. The first is always angle of attack in radians.

        Returns
        -------
        float
            Drag coefficient.
        """

        self.get_CD = np.vectorize(CD, doc=CD_docstring, excluded={0})

        def Cm(self, inputs):
            if self._type == "linear":
                return self._Cma*args[0]+self._CmL0
        
        Cm_docstring = """Returns the coefficient of moment as a function of args.

        Parameters
        ----------
        *args : floats
            Arbitrary airfoil parameters. The first is always angle of attack in radians

        Returns
        -------
        float
            Moment coefficient.
        """

        self.get_Cm = np.vectorize(Cm, doc=Cm_docstring, excluded={0})


    def get_CLa(self):
        """Returns the lift slope.

        Returns
        -------
        float
            Lift slope in 1/radians.
        """
        if self._type == "linear":
            return self._CLa


    def get_aL0(self):
        """Returns the zero-lift angle of attack.

        Returns
        -------
        float
            Zero-lift angle of attack in radians.
        """
        if self._type == "linear":
            return self._aL0
