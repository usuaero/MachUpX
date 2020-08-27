import airfoil_db as adb
import sys
import numpy as np

if __name__=="__main__":

    # Loop through arguments
    for NACA in sys.argv[1:]:

        # Input
        airfoil_input = {
            "geometry" : {
                "NACA" : NACA
            }
        }

        # Initialize
        airfoil = adb.Airfoil("NACA_"+NACA, airfoil_input)

        # Dofs
        DOFs = {
            "alpha" : {
                "range" : [np.radians(-20.0), np.radians(20.0)],
                "steps" : 21,
                "index" : 1
            },
            "Rey" : {
                "range" : [5e5, 1e7],
                "steps" : 20,
                "index" : 2,
                "log_step" : True
            }
        }

        # Generate database
        airfoil.generate_database(degrees_of_freedom=DOFs)
        airfoil.export_database(filename="dev/NACA_{0}_database.txt".format(NACA))

        # Generate fits
        airfoil.generate_polynomial_fit(CL_degrees="auto", CD_degrees="auto", Cm_degrees="auto")
        airfoil.export_polynomial_fits(filename="dev/NACA_{0}_fits.json".format(NACA))