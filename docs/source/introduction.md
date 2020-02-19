# Introduction
MachUpX is an implementation of Phillips' numerical lifting-line algorithm with some small modifications to increase its accuracy and versatility. For reference on numerical lifting-line, see Phillips and Snyder "Modern Adaptation of Classic Lifting-Line Theory". The purpose of numerical lifting-line is to quickly and accurately model the aerodynamics of fixed-wing aircraft. Within the limitations of potential flow theory, this algorithm produces very accurate results without the computational overhead of higher-order methods, such as vortex lattice method or CFD.

Those users already familiar with previous versions of MachUp (Pro/4/5/Py) will find the interface with MachUpX to be similar. However, attempts have not been made to ensure compatibility between previous versions of MachUp and MachUpX. If the user wishes to use analysis scripts/input files from previous versions of MachUp with MachUpX, modifications will have to be made. For this reason, significant effort has been made to provide clear and thorough documentation for MachUpX.

Examples for using the MachUpX API/command line interface can be found in the examples/ directory of the source code.

Help on using MachUpX can be found at the MachUp discussion forum on [Google Groups](https://groups.google.com/forum/#!categories/machup_forum).