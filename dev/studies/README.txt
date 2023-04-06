About the README File Template

Data deposited into DigitalCommons are required to have a README file that provides enough information about your research to enable users to use the data in their own research.

The more information you provide, the easier it will be for you or someone else to reuse your data in the future.  

Complete all fields that pertain to your data, format so it is easy to read, and save in a plain text format.

1. Dataset Title: Study into the Sensitity of the G-H Method to Blending Distance. Accompanies 
"Modern Implementation and Evaluation of Lifting-Line Theory for Complex Geometries", published
in AIAA Journal of Aircraft.

2. Name and contact information of PI:
a. Cory Goates
b. Utah State University
c. 4130 Old Main Hill, Logan, UT
d. cory.goates@gmail.com
e. 0000-0002-7568-1280

3. Name and contact information of Co-PI:
a. Doug Hunsaker
b. Utah State University
c. 4130 Old Main Hill, Logan, UT
d. doug.hunsaker@usu.edu

4. Funding source: This work was funded by the U.S. Office of Naval Research Sea-Based Aviation program (Grant No. N00014-18-1-
2502) with Brian Holm-Hansen as the program officer.

5. Project summary, description or abstract: 
A numerical lifting-line method (implemented in an open-source software package) is pre-
sented which can accurately estimate the aerodynamics of wings with arbitrary sweep, dihedral,
and twist. Previous numerical lifting-line methods have suffered from grid convergence chal-
lenges and limitations in accurately modeling the effects of sweep, or have relied on empirical
relations for swept-wing parameters and have been limited in their application to typical wing
geometries. This work presents novel improvements in accuracy, flexibility, and speed for
complex geometries over previous methods. In the current work, thin-airfoil theory is used to
correct section lift coefficients for sweep, providing a more general closure to the lifting-line
problem. A linearized solution is presented, which can be used as a rapid approximation for the
full solution, or as an initial guess for the nonlinear system of equations to speed convergence.
Sensitivities to model parameters are investigated, and appropriate recommendations for these
parameters are given. Agreement with Prandtl’s classical lifting-line method is excellent in the
case of straight wings. Comparison with experimental data shows this method can reasonably
predict lift, drag, and lift distribution for a range of wing configurations. The speed and
accuracy of this method make it well-suited for preliminary design and optimization.

7. Brief description of collection and processing of data: Data was generated using MachUpX (github.com/usuaero/MachUpX)

8. Description of files (names, or if too numerous, number of files, file type(s):
The archive contains two directories: "blending_distance_data" and "blending_distance_plots". The first contains
CSV files of the relevant data. Each CSV file is for a wing with given sweep type, sweep angle, taper ratio, and
aspect ratio (as indicated in the file name). The data are total lift coefficients as a function of grid resolution
and blending distance.

The second directory contains plots of the data. In addition to plotting, the raw CL values, the errors between the CL
values and the converged value (calculated using Richardson extrapolation) are plotted as well. The CL plots begin
with "CL_" and the error plots begin with "error_".

9. Descriptions of parameters/variables
a. Grid resolution: Number of discrete segments used per semispan on the wing.
b. Blending distance: Blending distance parameter input into the G-H numerical lifting-line method.
c. CL: Lift coefficient.
d. RT: Taper ratio, defined as the ratio of the tip chord to the root chord.
e. RA: Aspect ratio, defined as the wing planform area divided by the wingspan.
f. Sweep angle: Sweep angle at the tip of the wing.

10. Publications that cite or use this data:
Goates, C D, Hunsaker, D F, "Modern Implementation and Evaluation of Lifting-Line Theory for Complex Geometries," AIAA Journal of
Aircraft.