# Changes Between Version 1 and Version 2
After the development of MachUpX 1, we found that significant improvements could be made to the user interface and code structure. As such, we have made some changes switching to Version 2. These changes mean that input files written for Version 1 may no longer run properly for Version 2. However, minimal effort should be required to update. The various changes are explained below.

## Airfoil Database
MachUpX version 2.0 requires the AirfoilDatabase package, distributed by the USU AeroLab. It can be downloaded [here](www.githbu.com/usuaero/AirfoilDatabase).

## Aicraft State Specification
Version 1 of MachUpX supports two state input types, 'aerodynamic' and 'rigid_body'. Essentially, the only difference between these two was how the velocity was specified. To simplify things, we have removed the requirement to specify whether the input state is 'aerodynamic' or 'rigid_body'. Instead, the user must simply give the velocity in one of two ways. The first method is to give the magnitude of the aircraft's velocity and specify 'alpha' and 'beta' as well. If 'alpha' or 'beta' are not specified, they are assumed to be zero. The second method is to give the body-fixed velocity components of the aircraft (i.e. u, v, and w). These should be given as a list to the 'velocity' key. Version 2 will no longer support inputting the Earth-fixed velocity components. Please note that a velocity specified using a magnitude and alpha and beta will take into account wind, if such is present in the scene. If there is wind, then 'alpha' and 'beta' will be true aerodynamic angles relative to the local wind vector at the aircraft's origin.

As a whole, the USU AeroLab has decided to consistently use the sideslip angle defined as asin(Vy/V) in its analysis. MachUpX has been updated to reflect this change.

## Function Arguments
To simplify code structure, most of the scene member methods now use keyword arguments (kwargs as they are called in Python). You can read more about kwargs [here](https://book.pythontips.com/en/latest/args_and_kwargs.html). This may cause some scene method calls to break. In most cases, this can be fixed by formatting the method arguments as kwargs.
