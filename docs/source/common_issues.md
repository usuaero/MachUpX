# Common Issues
This page is for explaining common issues users face and ways to avoid them

## Poor Nonlinear Convergence
The nonlinear solver not converging quickly (or at all) can be caused by a number of things. Note, for me right now, the nonlinear solver usually takes ~35 iterations to converge. The following have been found to help:

* If Reid corrections are being used, make sure you have properly specified "wing_ID" under "grid" for each wing. Reid's corrections involve making adjustments to the locus of aerodynamic centers and so MachUpX needs to know which wing segments actually belong to the same wing.
* If multiple lifting surfaces come together at a point which are not technically part of the same wing (i.e. vertical and horizontal stabilizers), it can help to offset them slightly (on the order of 0.0001) from each other.