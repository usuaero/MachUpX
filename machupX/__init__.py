# Set environment variables to tell numpy to not use multithreading
import os
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['NUMEXPR_NUM_THREADS'] = '1'
os.environ['MKL_NUM_THREADS'] = '1'

# Other initialization
from machupX.scene import Scene
from machupX.exceptions import SolverNotConvergedError
from airfoil_db import DatabaseBoundsError