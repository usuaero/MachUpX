"""Code for Richardson extrapolation of numerical lifting-line results."""
import numpy as np

def richardson_extrap(h, I):
    """Performs Richardson extrapolation on the integrated quantities I obtained using the grid sizes h. Note that h is the number of discrete elements, not their size. The last three elements of h and I will be used, assuming that the last element of each corresponds to the most refined result. It is also assumed that h[-1]/h[-2] = h[-2]/h[-3].
    """

    # Estimate order of convergence
    p = np.log(np.abs((I[-3]-I[-2])/(I[-2]-I[-1])))/np.log(h[-1]/h[-2])

    # Get extrapolation
    I_ex = I[-1]+(I[-1]-I[-2])/((h[-1]/h[-2])**p-1.0)

    return I_ex, p