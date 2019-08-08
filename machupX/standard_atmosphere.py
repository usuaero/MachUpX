# A class defining a standard atmosphere profile based on the 1976 US Standard Atmosphere equations

import numpy as np
import copy

class StandardAtmosphere:
    """Defines a standard atmosphere according to the 1976 US Standard Atmosphere

    Parameters
    ----------
    unit_sys : str
        "English" or "SI"

    atmos_type : str
        Defaults to standard.
    """

    def __init__(self, unit_sys, atmos_type="standard"):

        # Import params
        self._unit_sys = unit_sys
        if atmos_type == "standard":
            self._T_0 = 15

        # Define constants. These are all defined in SI
        self._H_b = np.array([0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0, 84852.0]) # m
        self._L_M_b = np.array([-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002]) # K/m
        self._T_M_b = np.array([0.0, -71.5, -71.5, -59.5, -17.5, -17.5, -73.5, -101.204]) # K
        self._r_0 = 6356766.0 # Radius of the earth in meters
        self._g_0_prime = 9.80665 # m/s^2
        self._M_0 = 28.9644 # kg/kmol
        self._R_star = 8.31432e3 # N*m/kmol*K
        self._P_0 = 1.01325e5 # N/m^2
        self._S = 110.4 # K
        self._beta = 1.458e-6 # kg/s*m*K^1/2
        self._gamma = 1.40


    def T(self, h):
        """Returns the atmospheric temperature at the given geometric height.
        Either in Kelvin or Rankine
        """
        H = self._geometric_to_geopotential(h)
        T = np.interp(H, self._H_b, self._T_M_b)+self._T_0+273.15

        if self._unit_sys == "English":
            return 9.0/5.0*T
        else:
            return T


    def P(self, h):
        """Returns the atmospheric pressure at the given geometric height.
        Either in Pascals or lbf/ft^2.
        """
        H = self._geometric_to_geopotential(h)

        if isinstance(H, float):
            single = True
            H_array = np.asarray(H)[np.newaxis]
        else:
            single = False
            H_array = np.asarray(H)

        P = np.ones(H_array.shape)*self._P_0
        for i, H in enumerate(H_array):
            b = 0

            while b < 6 and H > self._H_b[b]:
                T_M_b = self._T_0+self._T_M_b[b]+273.15 # Layer base temp
                H_lim = min(H, self._H_b[b+1])
                if abs(self._L_M_b[b])<1e-6:
                    P[i] *= np.exp((-self._g_0_prime*self._M_0*(H_lim-self._H_b[b])/(self._R_star*T_M_b)))
                else:
                    exponent = (self._g_0_prime*self._M_0)/(self._R_star*self._L_M_b[b])
                    P[i] *= (T_M_b/(T_M_b+self._L_M_b[b]*(H_lim-self._H_b[b])))**exponent

                b += 1

        if single:
            P = P.item()

        if self._unit_sys == "English":
            return P*0.02088543815038
        else:
            return P
    

    def rho(self, h):
        """Returns the atmospheric density at the given geometric height.
        Either in kg/m^3 or slugs/ft^3.
        """
        P = self.P(h)
        T = self.T(h)

        if self._unit_sys == "English":
            P /= 0.02088543815038
            T = T*5.0/9.0

        rho = P*self._M_0/(self._R_star*T)

        if self._unit_sys == "English":
            return rho*0.0019403203
        else:
            return rho


    def mu(self, h):
        """Returns the dynamic viscosity at the given geometric height.
        Either in N*s/m^2 or lbf*s/ft^2
        """
        T = self.T(h)

        if self._unit_sys == "English":
            T = T*5.0/9.0
            
        mu = self._beta*T**1.5/(T+self._S)

        if self._unit_sys == "English":
            return mu*0.020885434273039
        else:
            return mu


    def a(self, h):
        """Returns the speed of sound at the given geometric height.
        Either m/s or ft/s
        """
        T = self.T(h)

        if self._unit_sys == "English":
            T = (T-32)*5.0/9.0

        a = np.sqrt(self._gamma*self._R_star*T/self._M_0)

        if self._unit_sys == "English":
            return a/0.3048
        else:
            return a


    def nu(self, h):
        """Returns the kinematic viscosity at the given geometric height.
        Either in m^2/s or ft^2/s
        """
        return self.mu(h)/self.rho(h)


    def _geometric_to_geopotential(self, h):
        # Check the height is not too high
        if self._unit_sys == "SI" and (h>86000.0).any():
            raise IOError("Standard atmosphere only goes up to 86 km.")

        # Convert to SI if needs be
        if self._unit_sys == "English":
            Z = h*0.3048
        else:
            Z = h

        # Convert
        return (self._r_0*Z)/(self._r_0+Z)