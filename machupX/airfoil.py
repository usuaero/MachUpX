from .helpers import *

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


    def __init__(self, name, input_dict={}):

        self.name = name
        self._input_dict = input_dict
        self._type = self._input_dict.get("type", "linear")

        self._initialize_data()
        self._initialize_geometry()

    
    def _initialize_data(self):
        # Initializes the necessary data structures for the airfoil
        # Linear airfoils are entirely defined by coefficients and coefficient derivatives
        if self._type == "linear":

            # Load from file
            try:
                filename = self._input_dict["path"]
                check_filepath(filename, ".json")
                with open(filename, 'r') as airfoil_file_handle:
                    params = json.load(airfoil_file_handle)

            # Load from input dict
            except KeyError:
                params = self._input_dict

            # Save params
            self._aL0 = import_value("aL0", params, "SI", 0.0) # The unit system doesn't matter
            self._CLa = import_value("CLa", params, "SI", 2*np.pi)
            self._CmL0 = import_value("CmL0", params, "SI", 0.0)
            self._Cma = import_value("Cma", params, "SI", 0.0)
            self._CD0 = import_value("CD0", params, "SI", 0.0)
            self._CD1 = import_value("CD1", params, "SI", 0.0)
            self._CD2 = import_value("CD2", params, "SI", 0.0)
            self._CL_max = import_value("CL_max", params, "SI", np.inf)

            self._CLM = import_value("CLM", params, "SI", 0.0)
            self._CLRe = import_value("CLRe", params, "SI", 0.0)

        else:
            raise IOError("'{0}' is not an allowable airfoil type.".format(self._type))


    def _initialize_geometry(self):
        # Creates an array of outline points to use in generating .stl and .iges files

        geom_params = self._input_dict.get("geometry", {})
        n = geom_params.get("N", 80)

        # Check that there's only one geometry definition
        points = geom_params.get("outline_points", None)
        naca_des = geom_params.get("NACA", None)
        if points is not None and naca_des is not None:
            raise IOError("Outline points and a NACA designation may not be both specified for airfoil {0}.".format(self.name))

        # Check for user-given points
        if points is not None:

            if isinstance(points, str): # Filepath
                with open(points, 'r') as input_handle:
                    self._outline_points = np.genfromtxt(input_handle, delimiter=',')

            elif isinstance(points, list) and isinstance(points[0], list): # Array
                self._outline_points = np.array(points)

        # NACA definition
        elif naca_des is not None:

            # Cosine distribution of chord locations
            theta = np.linspace(-np.pi, np.pi, n)
            x = 0.5*(1-np.cos(theta))

            # 4-digit series
            if len(naca_des) == 4:
                m = float(naca_des[0])/100
                p = float(naca_des[1])/10
                t = float(naca_des[2:])/100

                # Thickness distribution
                y_t = 5*t*(0.2969*np.sqrt(x)-0.1260*x-0.3516*x**2+0.2843*x**3-0.1036*x**4) # Uses formulation to seal trailing edge

                # Camber line equations
                if abs(m)<1e-10 or abs(p)<1e-10: # Symmetric
                    y_c = np.zeros_like(x)
                    dy_c_dx = np.zeros_like(x)
                else:
                    y_c = np.where(x<p, m/p**2*(2*p*x-x**2), m/(1-p)**2*((1-2*p)+2*p*x-x**2))
                    dy_c_dx = np.where(x<p, 2*m/p**2*(p-x), 2*m/(1-p**2)*(p-x))

                # Outline points
                X = x-y_t*np.sin(np.arctan(dy_c_dx))*np.sign(theta)
                Y = y_c+y_t*np.cos(np.arctan(dy_c_dx))*np.sign(theta)

                self._outline_points = np.concatenate([X[:,np.newaxis], Y[:,np.newaxis]], axis=1)


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
        """Returns the lift slope with respect to Mach number

        Parameters
        ----------
        inputs : ndarray
            Parameters which can affect the airfoil coefficients. The first
            three are always alpha, Reynolds number, and Mach number. Fourth 
            is flap efficiency and fifth is flap deflection.

        Returns
        -------
        float
            Lift slope with respect to Mach number
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


    def get_outline_points(self):
        """Returns an array of outline points showing the geometry of the airfoil.

        Returns
        -------
        ndarray
            Outline points in airfoil coordinates.
        """
        if hasattr(self, "_outline_points"):
            return np.copy(self._outline_points)
        else:
            raise RuntimeError("The geometry has not been defined for airfoil {0}.".format(self.name))