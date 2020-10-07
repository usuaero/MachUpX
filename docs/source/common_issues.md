# Common Issues
This page is for explaining common issues users face and ways to avoid them

## Poor Nonlinear Convergence
The nonlinear solver not converging quickly (or at all) can be caused by a number of things. Note, for me right now, the nonlinear solver usually takes ~7-10 iterations to converge for a "traditional" airframe. Up to 70 iterations is not uncommon though for more unusual airframes. If you're using a nonlinear airfoil and are trying to model past stall, good luck ever getting it to converge. The following have been found to help:

* If Reid corrections are being used, make sure you have properly specified "wing_ID" under "grid" for each wing. Reid's corrections involve blending the lifting line for a given lifting surface and so MachUpX needs to know which wing segments actually belong to the same wing.
* If multiple lifting surfaces come together at a point which are not technically part of the same wing (i.e. vertical and horizontal stabilizers), it can help to offset them slightly (on the order of 0.0001) from each other.
* The convergence can be adjusted by using the "relaxation" parameter under "solver" in the input dictionary/JSON. This defaults to 1.0, which corresponds to accepting the full correction at each step. For highly nonlinear geometries or for cases near stall, it may help to set this to some value less than unity.
* Try to make the wing geometry as smooth as possible. For example, if you are using winglets, adding a linear distribution of dihedral at the wing tips to blend into the winglets will help the solution.

**Getting all NaN results is not a convergence issue!** If your solver does not converge, you will not get any results, unless you have changed the default error state.

## DatabaseBoundsError Message
This error can occur if you are using a nonlinear airfoil database for determining section coefficients. It is caused by the local angle of attack, Mach number, Reynolds number, flap deflection, or flap chord fraction falling outside the bounds of what is in the database. If you see this error, you need to expand your airfoil database to allow for greater variation in the problem parameter.

For example, let's say I generate a NACA 0012 airfoil database that ranges from 10 to -10 degrees angle of attack. I then use this database for an aircraft I model in MachUpX. I then set the aircraft angle of attack (i.e. the freestream angle of attack) to 10 degrees. I will likely get an error because, due to induced velocities, the angle of attack of a given section may be increased above 10 degrees. My database doesn't know how a NACA 0012 airfoil behaves at above 10 degrees angle of attack, and so it will simply return a NaN. To fix this, I should regenerate my database over a wider range, say from 15 to -15 degrees.

You can see exactly where the database interpolation is failing using a ```try...except``` statement, like the following.

```python
try:
    scene.solve_forces()
except airfoil_db.DatabaseBoundsError as e:
    print(e.airfoil)
    print(e.exception_indices)
    print(e.inputs_dict)
```

## Trailing Vortex Impingement
MachUpX may give the following warning:
```
MachUpX detected a trailing vortex impinging upon a control point. This can lead to greatly exaggerated induced velocities at the control point.
See "Common Issues" in the documentation for more information. This warning can be suppressed by reducing "impingement_threshold" in the solver parameters.
```
Other ways this error can manifest are

* Ridiculous local values for alpha (i.e. 60 degrees when the global angle of attack is 0.0 degrees) showing up in a DatabaseBoundsError.
* Nonlinear solver divergence.

![](reid\ corrections.pdf)

Let me describe what is happening, as you may be able to recognize that your particular case may encourage this issue. Numerical lifting-line models each wing as a set of horseshoe vortices, as shown in the above figure. Each vortex is a singularity in potential flow and the velocity induced by each vortex goes to infinity as one approaches the vortex. As such, if a control point on an aft lifting surface happens to be too close to a trailing vortex from a forward lifting surface, the induced velocity predicted at that control point can be extremely high, leading to poor results or even failure to produce results.

The above warning will be given if in any case the denominator of the induced velocity equation for the trailing vortex segments falls below a certain threshold (1e-10 by default). If this warning is being given erroneously, the user can reduce the threshold by setting "impingement_threshold" in the solver dict within the input to MachUpX.

To fix the overall issue, it is best to offset one's geometry slightly to keep aft lifting surfaces out of the wake of forward lifting surfaces. This is a known failing of numerical lifting-line theory and is present in all versions of MachUp. Annoying, yes. I'm currently pondering other solutions...