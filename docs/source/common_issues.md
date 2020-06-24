# Common Issues
This page is for explaining common issues users face and ways to avoid them

## Poor Nonlinear Convergence
The nonlinear solver not converging quickly (or at all) can be caused by a number of things. Note, for me right now, the nonlinear solver usually takes ~7-10 iterations to converge for a "traditional" airframe. The following have been found to help:

* If Reid corrections are being used, make sure you have properly specified "wing_ID" under "grid" for each wing. Reid's corrections involve making adjustments to the locus of aerodynamic centers and so MachUpX needs to know which wing segments actually belong to the same wing.
* If multiple lifting surfaces come together at a point which are not technically part of the same wing (i.e. vertical and horizontal stabilizers), it can help to offset them slightly (on the order of 0.0001) from each other.
* The convergence can be adjusted by using the "relaxation" parameter under "solver" in the input dictionary/JSON. This defaults to 1.0, which corresponds to accepting the full correction at each step. For highly nonlinear geometries or for cases near stall, it may help to set this to some value less than unity.
* Try to make the wing geometry as smooth as possible. For example, if you are using winglets, adding a linear distribution of dihedral at the wing tips to blend into the winglets will help the solution.

**Getting all NaN results is not a convergence issue!** This is most likely a problem with using a nonlinear airfoil database, discussed below.

## NaN Results and "The inputs to airfoil..." Error Message
This error can occur if you are using a nonlinear airfoil database for determining section coefficients. It is caused by the local angle of attack, Mach number, Reynolds number, flap deflection, or flap chord fraction falling outside the bounds of what is in the database. If you see this error, you need to expand your airfoil database to allow for greater variation in the problem parameter.

For example, let's say I generate a NACA 0012 airfoil database that ranges from 10 to -10 degrees angle of attack. I then use this database for an aircraft I model in MachUpX. I then set the aircraft angle of attack (i.e. the freestream angle of attack) to 10 degrees. I will likely get an error because, due to induced velocities, the angle of attack of a given section may be increased above 10 degrees. My database doesn't know how a NACA 0012 airfoil behaves at above 10 degrees angle of attack, and so it will simply return a NaN. To fix this, I should regenerate my database over a wider range, say from 15 to -15 degrees.