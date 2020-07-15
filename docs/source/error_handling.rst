Error Handling
==============

The following custom errors are defined in MachUpX

.. automodule:: machupX
.. autoclass:: SolverNotConvergedError
   :members:

.. automodule:: airfoil_db
.. autoclass:: DatabaseBoundsError
   :members:

Errors can be processed as warnings or entirely suppressed using the Scene.set_err_state() method.