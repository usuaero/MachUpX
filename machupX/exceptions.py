"""Custom exceptions used in MachUpX."""


class SolverNotConvergedError(Exception):
    """An exception thrown when the solver fails to converge for a given case.

    Members
    ----------
    solver_type : str
        Either "nonlinear" or "scipy_fsolve".

    final_error : float
        The norm of the final error when computation was terminated.

    message : str
        A message about the error.
    """

    def __init__(self, solver_type, final_error):

        # Store args
        self.solver_type = solver_type
        self.final_error = final_error

        # Determine message
        if self.solver_type == "scipy_fsolve":
            self.message = "The scipy solver failed to converge, after which the nonlinear solver also failed to converge. Try relaxing the solver, increasing the maximum iterations, or increasing the convergence threshold."
        else:
            self.message = "The nonlinear solver failed to converge. Try relaxing the solver, increasing the maximum iterations, or increasing the convergence threshold."

        # Initialize super
        super().__init__(self.message)

    
    def __str__(self):
        return self.message+" The final error was {0}.".format(self.final_error)


class MaxIterationError(Exception):
    """An exception thrown when an iterative solver (such as pitch trim, not the nonlinear lifting-line solver) fails to converge within the maximum iterations.

    Members
    ----------
    throwing_function : str
        Member function of the Scene class which threw this exception.

    final_error : float
        Residual error of the solver at the time nonconvergence was determined.

    message : str
        A message about the error.
    """

    def __init__(self, throwing_function, final_error):

        # Store args
        self.throwing_function = throwing_function
        self.final_error = final_error
        self.message = "The solver within {0} failed to converge. The target state may not be attainable on this aircraft.".format(self.throwing_function)

        # Initialize super
        super().__init__(self.message)

    
    def __str__(self):
        return self.message+" The final error was {0}.".format(self.final_error)