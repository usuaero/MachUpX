Introduction
============

MachUpX is an implementation of Phillips' numerical lifting-line algorithm with some small modifications to increase its accuracy and versatility. The purpose of numerical lifting-line is to quickly and accurately model the aerodynamics of fixed-wing aircraft. Within the limitations of potential flow theory (i.e. high Reynolds number, not stalled), this algorithm produces very accurate results without the computational overhead of higher-order methods, such as vortex lattice method or CFD.

Examples for using the MachUpX API can be found in the examples/ directory of the source code.

All functionality of MachUpX is accessed through the Scene class. Accessing lower-level classes directly may cause unexpected behavior.