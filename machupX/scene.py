from .helpers import quat_inv_trans, quat_trans, check_filepath, import_value, quat_mult, quat_conj
from .airplane import Airplane
from .standard_atmosphere import StandardAtmosphere
from .exceptions import SolverNotConvergedError

import json
import time
import copy
import warnings

import numpy as np
import math as m
import scipy.interpolate as sinterp
import scipy.optimize as sopt
import matplotlib.pyplot as plt

from stl import mesh
from mpl_toolkits.mplot3d import Axes3D
from airfoil_db import DatabaseBoundsError

class Scene:
    """A class defining a scene containing one or more aircraft.

    Parameters
    ----------
    scene_input : string or dict, optional
        Dictionary or path to the JSON object specifying the scene parameters (see
        'Creating Input Files for MachUp'). If not specified, all default values are chosen.

    Raises
    ------
    IOError
        If input filepath or filename is invalid
    """

    def __init__(self, scene_input={}):

        # Initialize basic storage objects
        self._airplanes = {}
        self._N = 0
        self._num_aircraft = 0

        # Track whether the scene in its current state has been solved
        # Should be set to False any time any state variable is changed without immediately thereafter calling solve_forces()
        self._solved = False

        # Import information from the input
        self._load_params(scene_input)

        # Set the error handling state
        self.set_err_state()


    def _load_params(self, scene_input):
        # Loads JSON object and stores input parameters and aircraft

        # File
        if isinstance(scene_input, str):
            check_filepath(scene_input,".json")
            with open(scene_input) as input_json_handle:
                self._input_dict = json.load(input_json_handle)

        # Dictionary
        elif isinstance(scene_input, dict):
            self._input_dict = copy.deepcopy(scene_input)

        # Input format not recognized
        else:
            raise IOError("Input to Scene class initializer must be a file path or Python dictionary, not type {0}.".format(type(scene_input)))

        # Store solver parameters
        solver_params = self._input_dict.get("solver", {})
        self._solver_type = solver_params.get("type", "nonlinear")
        self._solver_convergence = solver_params.get("convergence", 1e-10)
        self._solver_relaxation = solver_params.get("relaxation", 1.0)
        self._max_solver_iterations = solver_params.get("max_iterations", 100)
        self._use_swept_sections = solver_params.get("use_swept_sections", True)
        self._use_total_velocity = solver_params.get("use_total_velocity", True)
        self._use_in_plane = solver_params.get("use_in_plane", True)
        self._match_machup_pro = solver_params.get("match_machup_pro", False)

        # Store unit system
        self._unit_sys = self._input_dict.get("units", "English")

        # Setup atmospheric property getter functions
        scene_dict = self._input_dict.get("scene", {})
        atmos_dict = scene_dict.get("atmosphere", {})
        self._std_atmos = StandardAtmosphere(unit_sys=self._unit_sys)
        self._get_density = self._initialize_density_getter(**atmos_dict)
        self._get_wind = self._initialize_wind_getter(**atmos_dict)
        self._get_viscosity = self._initialize_viscosity_getter(**atmos_dict)
        self._get_sos = self._initialize_sos_getter(**atmos_dict)

        # Initialize aircraft geometries
        aircraft_dict = scene_dict.get("aircraft", {})
        for key in aircraft_dict:

            # Get inputs
            airplane_file = self._input_dict["scene"]["aircraft"][key]["file"]
            state = self._input_dict["scene"]["aircraft"][key].get("state",{})
            control_state = self._input_dict["scene"]["aircraft"][key].get("control_state",{})

            # Instantiate
            self.add_aircraft(key, airplane_file, state=state, control_state=control_state)


    def _initialize_density_getter(self, **kwargs):

        # Load value from dictionary
        default_density = self._std_atmos.rho(0.0)
        rho = import_value("rho", kwargs, self._unit_sys, default_density)

        # Constant value
        if isinstance(rho, float):
            self._constant_rho = rho
            def density_getter(position):
                return self._constant_rho

        # Atmospheric table name
        elif isinstance(rho, str):
            # Profile
            if not rho in ["standard"]:
                raise IOError("{0} is not an allowable profile name.".format(rho))

            def density_getter(position):
                pos = position.T
                return self._std_atmos.rho(-pos[2])
            
        # Array
        elif isinstance(rho, np.ndarray):
            self._density_data = rho

            # Create getters
            if self._density_data.shape[1] is 2: # Density profile

                def density_getter(position):
                    pos = position.T
                    return np.interp(-pos[2], self._density_data[:,0], self._density_data[:,1])

            elif self._density_data.shape[1] is 4: # Density field
                self._density_field_interpolator = sinterp.LinearNDInterpolator(self._density_data[:,:3],self._density_data[:,3])

                def density_getter(position):
                    return self._density_field_interpolator(position)

        # Improper specification
        else:
            raise IOError("Density improperly specified as {0}.".format(rho))

        return density_getter

    
    def _initialize_wind_getter(self, **kwargs):
        
        # Load value from dict
        default_wind = [0.0, 0.0, 0.0]
        V_wind = import_value("V_wind", kwargs, self._unit_sys, default_wind)

        # Store wind
        if isinstance(V_wind, np.ndarray):

            if V_wind.shape == (3,): # Constant wind vector
                self._constant_wind = V_wind

                def wind_getter(position):
                    return self._constant_wind*np.ones(position.shape)

            else: # Array
                self._wind_data = V_wind
                
                # Create getters
                if self._wind_data.shape[1] is 6: # Wind field
                    self._wind_field_x_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3], self._wind_data[:,3], fill_value=0.0)
                    self._wind_field_y_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3], self._wind_data[:,4], fill_value=0.0)
                    self._wind_field_z_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3], self._wind_data[:,5], fill_value=0.0)

                    def wind_getter(position):
                        single = len(position.shape)==1
                        Vx = self._wind_field_x_interpolator(position)
                        Vy = self._wind_field_y_interpolator(position)
                        Vz = self._wind_field_z_interpolator(position)
                        if single:
                            return np.array([Vx.item(), Vy.item(), Vz.item()])
                        else:
                            return np.array([Vx, Vy, Vz]).T

                elif self._wind_data.shape[1] is 4: # wind profile

                    def wind_getter(position):
                        single = len(position.shape)==1
                        pos_T = position.T
                        Vx =  np.interp(-pos_T[2], self._wind_data[:,0], self._wind_data[:,1])
                        Vy =  np.interp(-pos_T[2], self._wind_data[:,0], self._wind_data[:,2])
                        Vz =  np.interp(-pos_T[2], self._wind_data[:,0], self._wind_data[:,3])
                        if single:
                            return np.array([Vx.item(), Vy.item(), Vz.item()])
                        else:
                            return np.array([Vx, Vy, Vz]).T

                else:
                    raise IOError("Wind array has the wrong number of columns.")
        
        else:
            raise IOError("Wind velocity improperly specified as {0}".format(V_wind))

        return wind_getter


    def _initialize_viscosity_getter(self, **kwargs):

        # Load value from dictionary
        default_visc = self._std_atmos.nu(0.0)
        nu = import_value("viscosity", kwargs, self._unit_sys, default_visc)

        # Constant value
        if isinstance(nu, float):
            self._constant_nu = nu
            def viscosity_getter(position):
                return self._constant_nu*np.ones((position.shape[:-1]))

        # Atmospheric profile name
        elif isinstance(nu, str):

            # Check we have that profile
            if not nu in ["standard"]:
                raise IOError("{0} is not an allowable profile name.".format(nu))

            def viscosity_getter(position):
                pos = np.transpose(position)
                return self._std_atmos.nu(-pos[2])

        return viscosity_getter


    def _initialize_sos_getter(self, **kwargs):

        # Load value from dictionary
        default_sos = self._std_atmos.a(0.0)
        a = import_value("speed_of_sound", kwargs, self._unit_sys, default_sos)

        # Constant value
        if isinstance(a, float):
            self._constant_a = a
            def sos_getter(position):
                return self._constant_a*np.ones((position.shape[:-1]))

        # Atmospheric profile name
        elif isinstance(a, str):

            # Check we have that profile
            if not a in ["standard"]:
                raise IOError("{0} is not an allowable profile name.".format(a))

            def sos_getter(position):
                pos = np.transpose(position)
                return self._std_atmos.a(-pos[2])

        return sos_getter


    def add_aircraft(self, airplane_name, airplane_input, state={}, control_state={}):
        """Inserts an aircraft into the scene. Note if an aircraft was specified
        in the input object, it has already been added to the scene.

        Parameters
        ----------
        airplane_name : str
            Name of the airplane to be added.

        airplane_input : str or dict
            JSON object (path) or dictionary describing the airplane.

        state : dict
            Dictionary describing the state of the airplane.

        control_state : dict
            Dictionary describing the state of the controls.
        """

        # Determine the local wind vector for setting the state of the aircraft
        aircraft_position = np.array(state.get("position", [0.0, 0.0, 0.0]))
        v_wind = self._get_wind(aircraft_position)

        # Create and store the aircraft object
        self._airplanes[airplane_name] = Airplane(airplane_name, airplane_input, self._unit_sys, self, init_state=state, init_control_state=control_state, v_wind=v_wind)

        # Update member variables
        self._N += self._airplanes[airplane_name].N
        self._num_aircraft += 1

        # Update geometry
        self._initialize_storage_arrays()
        self._store_aircraft_properties()
        self._perform_geometry_and_atmos_calcs()


    def remove_aircraft(self, airplane_name):
        """Removes an aircraft from the scene.

        Parameters
        ----------
        airplane_name : str
            Name of the airplane to be removed.
        """

        # Remove aircraft from dictionary
        try:
            deleted_aircraft = self._airplanes.pop(airplane_name)
        except KeyError:
            raise RuntimeError("The scene has no aircraft named {0}.".format(airplane_name))

        # Update quantities
        self._N -= deleted_aircraft.get_num_cps()
        self._num_aircraft -= 1

        # Reinitialize arrays
        if self._num_aircraft != 0:
            self._initialize_storage_arrays()
            self._store_aircraft_properties()
            self._perform_geometry_and_atmos_calcs()


    def _initialize_storage_arrays(self):
        # Initialize arrays

        # Section geometry
        self._c_bar = np.zeros(self._N) # Average chord
        self._dS = np.zeros(self._N) # Differential planform area
        self._PC = np.zeros((self._N,3)) # Control point location
        self._r_CG = np.zeros((self._N,3)) # Radii from airplane CG to control points
        self._dl = np.zeros((self._N,3)) # Differential LAC elements
        self._section_sweep = np.zeros(self._N)

        # Node locations
        self._P0 = np.zeros((self._N,self._N,3)) # Inbound vortex node location; takes into account effective LAC where appropriate
        self._P0_joint = np.zeros((self._N,self._N,3)) # Inbound vortex joint node location
        self._P1 = np.zeros((self._N,self._N,3)) # Outbound vortex node location
        self._P1_joint = np.zeros((self._N,self._N,3)) # Outbound vortex joint node location

        # Spatial node vectors and magnitudes
        self._r_0 = np.zeros((self._N,self._N,3))
        self._r_1 = np.zeros((self._N,self._N,3))
        self._r_0_joint = np.zeros((self._N,self._N,3))
        self._r_1_joint = np.zeros((self._N,self._N,3))
        self._r_0_mag = np.zeros((self._N,self._N))
        self._r_0_joint_mag = np.zeros((self._N,self._N))
        self._r_1_mag = np.zeros((self._N,self._N))
        self._r_1_joint_mag = np.zeros((self._N,self._N))

        # Spatial node vector magnitude products
        self._r_0_r_0_joint_mag = np.zeros((self._N,self._N))
        self._r_0_r_1_mag = np.zeros((self._N,self._N))
        self._r_1_r_1_joint_mag = np.zeros((self._N,self._N))

        # Section unit vectors
        self._u_a = np.zeros((self._N,3))
        self._u_n = np.zeros((self._N,3))
        self._u_s = np.zeros((self._N,3))

        # Control point atmospheric properties
        self._rho = np.zeros(self._N) # Density
        self._nu = np.zeros(self._N) # Viscosity
        self._a = np.ones(self._N) # Speed of sound

        # Airfoil parameters
        self._Re = np.zeros(self._N) # Reynolds number
        self._M = np.zeros(self._N) # Mach number
        self._aL0 = np.zeros(self._N) # Zero-lift angle of attack
        self._CLa = np.zeros(self._N) # Lift slope
        self._CL = np.zeros(self._N) # Lift coefficient
        self._CD = np.zeros(self._N) # Drag coefficient
        self._Cm = np.zeros(self._N) # Moment coefficient

        # Velocities
        self._v_wind = np.zeros((self._N,3))
        self._v_inf = np.zeros((self._N,3)) # Control point freestream vector
        if self._match_machup_pro:
            self._v_inf_w_o_rotation = np.zeros((self._N,3)) # Control point freestream vector minus influence of aircraft rotation
        self._P0_joint_v_inf = np.zeros((self._N,3))
        self._P1_joint_v_inf = np.zeros((self._N,3))

        # Misc
        self._diag_ind = np.diag_indices(self._N)
        self._gamma = np.zeros(self._N)

        self._solved = False
        

    def _store_aircraft_properties(self):
        # Get properties of the aircraft that don't change with state

        index = 0
        self._airplane_objects = []
        self._airplane_slices = []

        # Loop through airplanes
        for _, airplane_object in self._airplanes.items():

            # Store airplane objects to make sure they are always accessed in the same order
            self._airplane_objects.append(airplane_object)

            # Section of the arrays belonging to this airplane
            airplane_N = airplane_object.N
            airplane_slice = slice(index, index+airplane_N)
            self._airplane_slices.append(airplane_slice)

            # Get properties
            self._c_bar[airplane_slice] = airplane_object.c_bar
            self._dS[airplane_slice] = airplane_object.dS
            self._section_sweep[airplane_slice] = airplane_object.section_sweep

            index += airplane_N

        # Swept section corrections based on thin airfoil theory
        if self._use_swept_sections:
            C_lambda = np.cos(self._section_sweep)
            self._c_bar *= C_lambda
            self._C_sweep_inv = 1.0/C_lambda

        self._solved = False


    def _perform_geometry_and_atmos_calcs(self):
        # Performs calculations necessary for solving NLL which are only dependent on geometry.
        # This speeds up repeated calls to _solve(). This method should be called any time the 
        # geometry is updated, an aircraft is added to the scene, or the position or orientation
        # of an aircraft changes. Note that all calculations occur in the Earth-fixed frame.

        # Loop through airplanes
        for airplane_object, airplane_slice in zip(self._airplane_objects, self._airplane_slices):

            # Get airplane
            q = airplane_object.q
            p = airplane_object.p_bar

            # Get geometries
            PC = quat_inv_trans(q, airplane_object.PC)
            self._r_CG[airplane_slice,:] = quat_inv_trans(q, airplane_object.PC_CG)
            self._PC[airplane_slice,:] = p+PC
            self._dl[airplane_slice,:] = quat_inv_trans(q, airplane_object.dl)

            # Get section vectors
            if self._use_swept_sections:
                self._u_a[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_a)
                self._u_n[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_n)
                self._u_s[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_s)
            else:
                self._u_a[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_a_unswept)
                self._u_n[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_n_unswept)
                self._u_s[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_s_unswept)

            # Node locations
            # Note the first index indicates which control point this is the effective LAC for
            self._P0[airplane_slice,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P0_eff)
            self._P1[airplane_slice,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P1_eff)
            self._P0_joint[airplane_slice,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P0_joint_eff)
            self._P1_joint[airplane_slice,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P1_joint_eff)

            # Get node locations for other aircraft from this aircraft
            # This does not need to take the effective LAC into account
            if self._num_aircraft > 1:
                this_ind = range(airplane_slice.start, airplane_slice.stop)
                other_ind = [i for i in range(self._N) if i not in this_ind] # control point indices for other airplanes
                self._P0[other_ind,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P0)
                self._P1[other_ind,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P1)
                self._P0_joint[other_ind,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P0_joint)
                self._P1_joint[other_ind,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P1_joint)

            # Spatial node vectors
            self._r_0[airplane_slice,airplane_slice,:] = quat_inv_trans(q, airplane_object.r_0)
            self._r_1[airplane_slice,airplane_slice,:] = quat_inv_trans(q, airplane_object.r_1)
            self._r_0_joint[airplane_slice,airplane_slice,:] = quat_inv_trans(q, airplane_object.r_0_joint)
            self._r_1_joint[airplane_slice,airplane_slice,:] = quat_inv_trans(q, airplane_object.r_1_joint)

            # Spatial node vector magnitudes
            self._r_0_mag[airplane_slice,airplane_slice] = airplane_object.r_0_mag
            self._r_0_joint_mag[airplane_slice,airplane_slice] = airplane_object.r_0_joint_mag
            self._r_1_mag[airplane_slice,airplane_slice] = airplane_object.r_1_mag
            self._r_1_joint_mag[airplane_slice,airplane_slice] = airplane_object.r_1_joint_mag

            # Spatial node vector magnitude products
            self._r_0_r_0_joint_mag[airplane_slice,airplane_slice] = airplane_object.r_0_r_0_joint_mag
            self._r_0_r_1_mag[airplane_slice,airplane_slice] = airplane_object.r_0_r_1_mag
            self._r_1_r_1_joint_mag[airplane_slice,airplane_slice] = airplane_object.r_1_r_1_joint_mag

        # Fill in spatial node vectors between airplanes
        if self._num_aircraft > 1:
            for airplane_slice in self._airplane_slices:
                this_ind = range(airplane_slice.start, airplane_slice.stop)
                other_ind = [i for i in range(self._N) if i not in this_ind] # control point indices for other airplanes

                # Spatial node vectors
                self._r_0[airplane_slice,other_ind,:] = self._PC[airplane_slice,np.newaxis,:]-self._P0[airplane_slice,other_ind,:]
                self._r_1[airplane_slice,other_ind,:] = self._PC[airplane_slice,np.newaxis,:]-self._P1[airplane_slice,other_ind,:]
                self._r_0_joint[airplane_slice,other_ind,:] = self._PC[airplane_slice,np.newaxis,:]-self._P0_joint[airplane_slice,other_ind,:]
                self._r_1_joint[airplane_slice,other_ind,:] = self._PC[airplane_slice,np.newaxis,:]-self._P1_joint[airplane_slice,other_ind,:]

                # Calculate spatial node vector magnitudes
                self._r_0_mag[airplane_slice,other_ind] = np.sqrt(np.einsum('ijk,ijk->ij', self._r_0[airplane_slice,other_ind,:], self._r_0[airplane_slice,other_ind,:]))
                self._r_0_joint_mag[airplane_slice,other_ind] = np.sqrt(np.einsum('ijk,ijk->ij', self._r_0_joint[airplane_slice,other_ind,:], self._r_0_joint[airplane_slice,other_ind,:]))
                self._r_1_mag[airplane_slice,other_ind] = np.sqrt(np.einsum('ijk,ijk->ij', self._r_1[airplane_slice,other_ind,:], self._r_1[airplane_slice,other_ind,:]))
                self._r_1_joint_mag[airplane_slice,other_ind] = np.sqrt(np.einsum('ijk,ijk->ij', self._r_1_joint[airplane_slice,other_ind,:], self._r_1_joint[airplane_slice,other_ind,:]))

                # Calculate magnitude products
                self._r_0_r_0_joint_mag[airplane_slice,other_ind] = self._r_0_mag[airplane_slice,other_ind]*self._r_0_joint_mag[airplane_slice,other_ind]
                self._r_0_r_1_mag[airplane_slice,other_ind] = self._r_0_mag[airplane_slice,other_ind]*self._r_1_mag[airplane_slice,other_ind]
                self._r_1_r_1_joint_mag[airplane_slice,other_ind] = self._r_1_mag[airplane_slice,other_ind]*self._r_1_joint_mag[airplane_slice,other_ind]

        # In-plane projection matrices
        if self._use_in_plane:
            self._P_in_plane = np.repeat(np.identity(3)[np.newaxis,:,:], self._N, axis=0)-np.matmul(self._u_s[:,:,np.newaxis], self._u_s[:,np.newaxis,:])

        # Influence of bound and jointed vortex segments
        with np.errstate(divide='ignore', invalid='ignore'):

            # Bound
            numer = ((self._r_0_mag+self._r_1_mag)[:,:,np.newaxis]*np.cross(self._r_0, self._r_1))
            denom = self._r_0_r_1_mag*(self._r_0_r_1_mag+np.einsum('ijk,ijk->ij', self._r_0, self._r_1))
            V_ji_bound = np.true_divide(numer, denom[:,:,np.newaxis])
            V_ji_bound[np.diag_indices(self._N)] = 0.0 # Ensure this actually comes out to be zero

            # Jointed 0
            numer = (self._r_0_joint_mag+self._r_0_mag)[:,:,np.newaxis]*np.cross(self._r_0_joint, self._r_0)
            denom = self._r_0_r_0_joint_mag*(self._r_0_r_0_joint_mag+np.einsum('ijk,ijk->ij', self._r_0_joint, self._r_0))
            V_ji_joint_0 = np.true_divide(numer, denom[:,:,np.newaxis])

            # Jointed 1
            numer = (self._r_1_joint_mag+self._r_1_mag)[:,:,np.newaxis]*np.cross(self._r_1, self._r_1_joint)
            denom = self._r_1_r_1_joint_mag*(self._r_1_r_1_joint_mag+np.einsum('ijk,ijk->ij', self._r_1, self._r_1_joint))
            V_ji_joint_1 = np.true_divide(numer, denom[:,:,np.newaxis])

            # Sum
            self._V_ji_const = V_ji_bound+V_ji_joint_0+V_ji_joint_1

        # Atmospheric wind, density, speed of sound, and viscosity
        self._rho = self._get_density(self._PC)
        self._a = self._get_sos(self._PC)
        self._nu = self._get_viscosity(self._PC)
        self._v_wind[:,:] = self._get_wind(self._PC)

        self._solved = False


    def _calc_invariant_flow_properties(self):
        # Calculates the invariant flow properties at each control point and node location. These are
        # dependent upon aircraft velocity and angular rate.

        # Loop through airplanes
        for airplane_object, airplane_slice in zip(self._airplane_objects, self._airplane_slices):

            # Freestream velocity due to airplane translation
            v_trans = -airplane_object.v
            w = airplane_object.w

            # Control point velocities
            v_rot = quat_inv_trans(airplane_object.q, -np.cross(w, airplane_object.PC_CG))
            v_wind = self._v_wind[airplane_slice]
            self._v_inf[airplane_slice,:] = v_trans+v_wind+v_rot
            if self._match_machup_pro:
                self._v_inf_w_o_rotation[airplane_slice,:] = v_trans+v_wind

            # Joint velocities for determining trailing vortex direction
            if self._match_machup_pro:
                self._P0_joint_v_inf[airplane_slice,:] = v_trans+v_wind
                self._P1_joint_v_inf[airplane_slice,:] = v_trans+v_wind
            else:
                P0_joint_v_rot = quat_inv_trans(airplane_object.q, -np.cross(w, airplane_object.P0_joint-airplane_object.CG[np.newaxis,:]))
                P1_joint_v_rot = quat_inv_trans(airplane_object.q, -np.cross(w, airplane_object.P1_joint-airplane_object.CG[np.newaxis,:]))
                self._P0_joint_v_inf[airplane_slice,:] = v_trans+v_wind+P0_joint_v_rot
                self._P1_joint_v_inf[airplane_slice,:] = v_trans+v_wind+P1_joint_v_rot

        # Get freestream magnitudes and directions
        self._V_inf = np.linalg.norm(self._v_inf, axis=1)
        self._u_inf = self._v_inf/self._V_inf[:,np.newaxis]
        if self._match_machup_pro:
            self._V_inf_w_o_rotation = np.linalg.norm(self._v_inf_w_o_rotation, axis=1)

        # Calculate nodal freestream unit vectors to determine the direction of the trailing vortices
        self._P0_joint_u_inf = self._P0_joint_v_inf/np.linalg.norm(self._P0_joint_v_inf, axis=-1, keepdims=True)
        self._P1_joint_u_inf = self._P1_joint_v_inf/np.linalg.norm(self._P1_joint_v_inf, axis=-1, keepdims=True)

        # Calculate V_ji
        # Influence of vortex segment 0 after the joint; ignore if the radius goes to zero
        denom = (self._r_0_joint_mag*(self._r_0_joint_mag-np.einsum('ijk,ijk->ij', self._P0_joint_u_inf[np.newaxis], self._r_0_joint)))
        V_ji_due_to_0 = np.nan_to_num(-np.cross(self._P0_joint_u_inf, self._r_0_joint)/denom[:,:,np.newaxis])

        # Influence of vortex segment 1 after the joint
        denom = (self._r_1_joint_mag*(self._r_1_joint_mag-np.einsum('ijk,ijk->ij', self._P1_joint_u_inf[np.newaxis], self._r_1_joint)))
        V_ji_due_to_1 = np.nan_to_num(np.cross(self._P1_joint_u_inf, self._r_1_joint)/denom[:,:,np.newaxis])

        # Sum
        # In my definition of V_ji, the first index is the control point, the second index is the horseshoe vortex, and the third index is the vector components
        self._V_ji = 1/(4*np.pi)*(V_ji_due_to_0+self._V_ji_const+V_ji_due_to_1)

        # Get effective freesream and calculate initial approximation for airfoil parameters (Re and M are only used in the linear solution)
        if self._use_in_plane:
            self._v_inf_in_plane = np.matmul(self._P_in_plane, self._v_inf[:,:,np.newaxis]).reshape((self._N,3))
            self._V_inf_in_plane = np.linalg.norm(self._v_inf_in_plane, axis=1)
            self._Re = self._V_inf_in_plane*self._c_bar/self._nu
            self._M = self._V_inf_in_plane/self._a
        else:
            self._Re = self._V_inf*self._c_bar/self._nu
            self._M = self._V_inf/self._a

        self._v_n_inf = np.einsum('ij,ij->i', self._v_inf, self._u_n)
        self._v_a_inf = np.einsum('ij,ij->i', self._v_inf, self._u_a)
        self._alpha_inf = np.arctan2(self._v_n_inf, self._v_a_inf)

        # Get lift slopes and zero-lift angles of attack for each segment
        for airplane_object, airplane_slice in zip(self._airplane_objects, self._airplane_slices):
            seg_ind = 0
            for segment in airplane_object.segments:
                seg_N = segment.N
                seg_slice = slice(airplane_slice.start+seg_ind, airplane_slice.start+seg_ind+seg_N)
                self._CLa[seg_slice] = segment.get_cp_CLa(self._alpha_inf[seg_slice], self._Re[seg_slice], self._M[seg_slice])
                self._CL[seg_slice] = segment.get_cp_CL(self._alpha_inf[seg_slice], self._Re[seg_slice], self._M[seg_slice])
                self._aL0[seg_slice] = segment.get_cp_aL0(self._Re[seg_slice], self._M[seg_slice])
                seg_ind += seg_N

        # Correct CL estimate for sweep (we don't use self._correct_CL_for_sweep() here because we are dealing with alpha_inf rather than true alpha)
        if self._use_swept_sections:

            # Estimate lift slope
            with np.errstate(divide='ignore', invalid='ignore'):
                self._CLa = np.nan_to_num(self._CL/(self._alpha_inf-self._aL0))

            # Get new estimate
            self._CL = self._CLa*(self._alpha_inf-self._aL0*self._C_sweep_inv)

        self._solved = False


    def _solve_w_scipy(self, **kwargs):
        # Determines the votrex strengths using scipy.fsolve

        # Initialize
        start_time = time.time()
        verbose = kwargs.get("verbose", False)
        if verbose: print("Running scipy solver...")

        # Set up flow for what won't change with changes in vorticity distribution
        self._calc_invariant_flow_properties()

        # Initial guess
        gamma_init = np.zeros(self._N)

        # Get solution
        self._gamma, info, ier, mesg = sopt.fsolve(self._lifting_line_residual, gamma_init, full_output=True)#, xtol=self._solver_convergence)

        # Output fsolve info
        if verbose:
            print("Complete!")
            print("   Number of function calls: {0}".format(info["nfev"]))
            print("   Norm of final residual vector: {0}".format(np.linalg.norm(info["fvec"])))

        # Check for no solution
        if verbose and ier != 1:
            print("Scipy.optimize.fsolve was unable to find a solution.")
            print("Error message: {0}".format(mesg))
            print("Norm of final residual vector: {0}".format(np.linalg.norm(info["fvec"])))
            print("Scipy solver failed. Reverting to nonlinear solution...")
            return -1

        return time.time()-start_time


    def _lifting_line_residual(self, gamma):
        # Returns the residual to nonlinear lifting-line equation

        # Set vorticity
        self._gamma = gamma

        # Calculate control point velocities
        self._calc_v_i()

        # Get vortex lift
        self._w_i = np.cross(self._v_i, self._dl)
        self._w_i_mag = np.linalg.norm(self._w_i, axis=1)
        L_vortex = 2.0*self._w_i_mag*self._gamma
        
        # Get section lift
        L_section = self._get_section_lift()

        # Return difference
        return L_vortex-L_section


    def _calc_v_i(self):
        # Determines the local velocity at each control point
        self._v_i = self._v_inf+(self._V_ji.transpose((2,0,1))@self._gamma).T

    
    def _get_section_lift(self):
        # Calculate magnitude of lift due to section properties divided by 1/2 density

        # Get section properties
        if self._use_in_plane:
            self._v_i_in_plane = np.matmul(self._P_in_plane, self._v_i[:,:,np.newaxis]).reshape((self._N,3))
            self._V_i_in_plane_2 = np.einsum('ij,ij->i', self._v_i_in_plane, self._v_i_in_plane)
            self._V_i_in_plane = np.sqrt(self._V_i_in_plane_2)

            self._Re = self._V_i_in_plane*self._c_bar/self._nu
            self._M = self._V_i_in_plane/self._a
        else:
            self._V_i_2 = np.einsum('ij,ij->i', self._v_i, self._v_i)
            self._V_i = np.sqrt(self._V_i_2)

            self._Re = self._V_i*self._c_bar/self._nu
            self._M = self._V_i/self._a

        # Calculate angle of attack
        self._v_a = np.einsum('ij,ij->i', self._v_i, self._u_a)
        self._v_n = np.einsum('ij,ij->i', self._v_i, self._u_n)
        self._alpha = np.arctan2(self._v_n, self._v_a)

        # Loop through airplanes
        index = 0
        for airplane_object in self._airplane_objects:
            N = airplane_object.N

            # Loop through segments
            seg_ind = 0
            for segment in airplane_object.segments:
                seg_N = segment.N
                seg_slice = slice(index+seg_ind, index+seg_ind+seg_N)
                self._CL[seg_slice] = segment.get_cp_CL(self._alpha[seg_slice], self._Re[seg_slice], self._M[seg_slice])
                seg_ind += seg_N

            index += N

        # Return lift to match MU Pro
        if self._match_machup_pro:
            return self._V_inf_w_o_rotation*self._V_inf_w_o_rotation*self._CL*self._dS

        # Correct lift coefficient
        if self._use_swept_sections:
            self._correct_CL_for_sweep()

        # Return lift coefficient based on certain conditions
        if self._use_total_velocity:
            if self._use_in_plane:
                return self._V_i_in_plane_2*self._CL*self._dS # in case you're wondering, this is the one you want to go for ;)
            else:
                return self._V_i_2*self._CL*self._dS
        else:
            if self._use_in_plane:
                return self._V_inf_in_plane*self._V_inf_in_plane*self._CL*self._dS
            else:
                return self._V_inf*self._V_inf*self._CL*self._dS


    def _correct_CL_for_sweep(self):
        # Applies thin-airfoil corrections for swept section lift

        # Estimate lift slope
        with np.errstate(divide='ignore', invalid='ignore'):
            self._CLa = np.nan_to_num(self._CL/(self._alpha-self._aL0))

        # Get new estimate
        self._CL = self._CLa*(self._alpha-self._aL0*self._C_sweep_inv)


    def _solve_linear(self, **kwargs):
        # Determines the vortex strengths of all horseshoe vortices in the scene using the linearized equations

        start_time = time.time()
        verbose = kwargs.get("verbose", False)
        if verbose: print("Running linear solver...")

        # Calculate invariant properties
        self._calc_invariant_flow_properties()

        # Calculate velocity cross product and b vector
        if self._use_in_plane:
            u_inf_x_dl = np.cross(self._v_inf_in_plane/self._V_inf_in_plane[:,np.newaxis], self._dl)
            b = self._V_inf_in_plane*self._dS*self._CL # Phillips and Hunsaker use CL here instead of CL,a(a-a_L0). It is more accurate for nonlinear airfoils.
        else:
            u_inf_x_dl = np.cross(self._u_inf, self._dl)
            b = self._V_inf*self._dS*self._CL

        # A matrix
        V_ji_dot_u_n = np.einsum('ijk,ik->ij', self._V_ji, self._u_n)
        A = np.zeros((self._N,self._N))
        A[:,:] = -(self._CLa*self._dS)[:,np.newaxis]*V_ji_dot_u_n
        A[self._diag_ind] += 2.0*np.linalg.norm(u_inf_x_dl, axis=1)

        # Solve
        self._gamma = np.linalg.solve(A, b)

        return time.time()-start_time


    def _solve_nonlinear(self, **kwargs):
        # Nonlinear improvement to the vector of gammas already determined
        verbose = kwargs.get("verbose", False)
        if verbose: 
            print("Running nonlinear solver...")
            print("    Relaxation: {0}".format(self._solver_relaxation))
            print("    Convergence: {0}".format(self._solver_convergence))
            print("{0:<20}{1:<20}".format("Iteration", "Error"))
            print("".join(['-']*40))
        self._nonlinear_start_time = time.time()

        J = np.zeros((self._N, self._N))

        # Airfoil coefs
        C_LRe = np.zeros(self._N)
        C_LM = np.zeros(self._N)

        # Calculate the derivative of induced velocity wrt vortex strength
        if self._use_in_plane:
            V_ji = np.matmul(self._P_in_plane, self._V_ji[:,:,:,np.newaxis]).reshape((self._N,self._N,3))
        else:
            V_ji = self._V_ji

        iteration = 0
        error = 100
        while error > self._solver_convergence:
            iteration += 1

            # Get residual vector (calculates alpha, V_i, CL, etc... for us)
            R = self._lifting_line_residual(self._gamma)
            error = np.linalg.norm(R)

            # Loop through airplanes
            index = 0
            for airplane_object in self._airplane_objects:

                # Loop through segments
                for segment_object in airplane_object.segments:
                    num_cps = segment_object.N
                    cur_slice = slice(index, index+num_cps)

                    # Get lift coefficient and lift slopes
                    self._CLa[cur_slice] = segment_object.get_cp_CLa(self._alpha[cur_slice], self._Re[cur_slice], self._M[cur_slice])
                    C_LRe[cur_slice] = segment_object.get_cp_CLRe(self._alpha[cur_slice], self._Re[cur_slice], self._M[cur_slice])
                    C_LM[cur_slice] = segment_object.get_cp_CLM(self._alpha[cur_slice], self._Re[cur_slice], self._M[cur_slice])

                    index += num_cps

            # Intermediate calcs
            if self._use_in_plane:
                v_iji = np.einsum('ijk,ijk->ij', self._v_i_in_plane[:,np.newaxis,:], V_ji)
            else:
                v_iji = np.einsum('ijk,ijk->ij', self._v_i[:,np.newaxis,:], V_ji)

            # Caclulate Jacobian
            J[:,:] = (2*self._gamma/self._w_i_mag)[:,np.newaxis]*(np.einsum('ijk,ijk->ij', self._w_i[:,np.newaxis,:], np.cross(V_ji, self._dl)))

            if self._use_total_velocity:
                J[:,:] -= (2*self._dS*self._CL)[:,np.newaxis]*v_iji # Comes from taking the derivative of V_i^2 with respect to gamma

            if self._use_in_plane:
                CL_gamma_Re = C_LRe[:,np.newaxis]*self._c_bar/(self._nu*self._V_i_in_plane)[:,np.newaxis]*v_iji
                CL_gamma_M = C_LM[:,np.newaxis]/(self._a*self._V_i_in_plane)[:,np.newaxis]*v_iji
            else:
                CL_gamma_Re = C_LRe[:,np.newaxis]*self._c_bar/(self._nu*self._V_i)[:,np.newaxis]*v_iji
                CL_gamma_M = C_LM[:,np.newaxis]/(self._a*self._V_i)[:,np.newaxis]*v_iji

            CL_gamma_alpha = self._CLa[:,np.newaxis]*(self._v_a[:,np.newaxis]*np.einsum('ijk,ijk->ij', V_ji, self._u_n[:,np.newaxis])-self._v_n[:,np.newaxis]*np.einsum('ijk,ijk->ij', V_ji, self._u_a[:,np.newaxis]))/(self._v_n*self._v_n+self._v_a*self._v_a)[:,np.newaxis]

            if self._use_total_velocity:
                if self._use_in_plane:
                    J[:,:] -= (self._V_i_in_plane_2*self._dS)[:,np.newaxis]*(CL_gamma_alpha+CL_gamma_Re+CL_gamma_M)
                else:
                    J[:,:] -= (self._V_i_2*self._dS)[:,np.newaxis]*(CL_gamma_alpha+CL_gamma_Re+CL_gamma_M)
            else:
                if self._use_in_plane:
                    J[:,:] -= (self._V_inf_in_plane*self._V_inf_in_plane*self._dS)[:,np.newaxis]*(CL_gamma_alpha+CL_gamma_Re+CL_gamma_M)
                else:
                    J[:,:] -= (self._V_inf*self._V_inf*self._dS)[:,np.newaxis]*(CL_gamma_alpha+CL_gamma_Re+CL_gamma_M)

            diag_ind = np.diag_indices(self._N)
            J[diag_ind] += 2*self._w_i_mag

            # Get gamma update
            dGamma = np.linalg.solve(J, -R)

            # Update gamma
            self._gamma = self._gamma+self._solver_relaxation*dGamma

            # Output progress
            if verbose: print("{0:<20}{1:<20}".format(iteration, error))

            # Check this isn't taking too long
            if iteration >= self._max_solver_iterations:
                R = self._lifting_line_residual(self._gamma)
                error = np.linalg.norm(R)
                raise SolverNotConvergedError(self._solver_type, error)

        # Loop exits normally
        else:
            R = self._lifting_line_residual(self._gamma)
            error = np.linalg.norm(R)
            if verbose:
                print("Nonlinear solver successfully converged. Final error: {0}".format(error))

        return time.time()-self._nonlinear_start_time


    def _get_frames(self, **kwargs):
        body_frame = kwargs.get("body_frame", True)
        stab_frame = kwargs.get("stab_frame", False)
        wind_frame = kwargs.get("wind_frame", True)
        return body_frame, stab_frame, wind_frame


    def _integrate_forces_and_moments(self, **kwargs):
        # Determines the forces and moments on each lifting surface
        start_time = time.time()

        # Kwargs
        non_dimensional = kwargs.get("non_dimensional", True)
        dimensional = kwargs.get("dimensional", True)
        report_by_segment = kwargs.get("report_by_segment", False)
        body_frame, stab_frame, wind_frame = self._get_frames(**kwargs)

        # Scale gammas to match MachUp Pro
        if self._match_machup_pro:
            self._gamma *= (self._V_inf/self._V_inf_w_o_rotation)**2

        # Get velocities
        if self._use_total_velocity or self._match_machup_pro:
            self._calc_v_i()
            self._V_i_2 = np.einsum('ij,ij->i', self._v_i, self._v_i)
            self._V_i = np.sqrt(self._V_i_2)
            self._u_i = self._v_i/self._V_i[:,np.newaxis]
            if self._use_in_plane:
                self._v_i_in_plane = np.matmul(self._P_in_plane, self._v_i[:,:,np.newaxis]).reshape((self._N,3))
                self._V_i_in_plane_2 = np.einsum('ij,ij->i', self._v_i_in_plane, self._v_i_in_plane)

        # Calculate vortex force differential elements
        self._dF_inv = (self._rho*self._gamma)[:,np.newaxis]*np.cross(self._v_i, self._dl)

        # Calculate conditions for determining viscid contributions
        self._v_a = np.einsum('ij,ij->i', self._v_i, self._u_a)
        self._v_n = np.einsum('ij,ij->i', self._v_i, self._u_n)
        self._alpha = np.arctan2(self._v_n, self._v_a)
        if self._use_in_plane:
            self._V_i_in_plane = np.sqrt(self._V_i_in_plane_2)
            self._Re = self._V_i_in_plane*self._c_bar/self._nu
            self._M = self._V_i_in_plane/self._a
        else:
            self._Re = self._V_i*self._c_bar/self._nu
            self._M = self._V_i/self._a

        # Redimensionalization parameters
        if self._use_total_velocity:
            self._redim_full = 0.5*self._rho*self._V_i_2*self._dS
            if self._use_in_plane:
                self._redim_in_plane = 0.5*self._rho*self._V_i_in_plane_2*self._dS
        else:
            self._redim_full = 0.5*self._rho*self._V_inf*self._V_inf*self._dS
            if self._use_in_plane:
                self._redim_in_plane = 0.5*self._rho*self._V_inf_in_plane*self._V_inf_in_plane*self._dS

        # Store lift, drag, and moment coefficient distributions
        empty_coef_dict = {}
        empty_FM_dict = {}
        if body_frame:
            empty_coef_dict.update({"Cx" : {}, "Cy" : {}, "Cz" : {}, "Cl" : {}, "Cm" : {}, "Cn" : {}})
            empty_FM_dict.update({"Fx" : {}, "Fy" : {}, "Fz" : {}, "Mx" : {}, "My" : {}, "Mz" : {}})
        if stab_frame:
            empty_coef_dict.update({"Cx_s" : {}, "Cy_s" : {}, "Cz_s" : {}, "Cl_s" : {}, "Cm_s" : {}, "Cn_s" : {}})
            empty_FM_dict.update({"Fx_s" : {}, "Fy_s" : {}, "Fz_s" : {}, "Mx_s" : {}, "My_s" : {}, "Mz_s" : {}})
        if wind_frame:
            empty_coef_dict.update({"CL" : {}, "CD" : {}, "CS" : {}, "Cl_w" : {}, "Cm_w" : {}, "Cn_w" : {}})
            empty_FM_dict.update({"FL" : {}, "FD" : {}, "FS" : {}, "Mx_w" : {}, "My_w" : {}, "Mz_w" : {}})

        # Get section moment and drag coefficients
        index = 0
        for airplane_object in self._airplane_objects:
            for segment in airplane_object.segments:
                num_cps = segment.N
                cur_slice = slice(index, index+num_cps)

                # Section drag coefficient
                self._CD[cur_slice] = segment.get_cp_CD(self._alpha[cur_slice], self._Re[cur_slice], self._M[cur_slice])

                # Section moment coefficient
                self._Cm[cur_slice] = segment.get_cp_Cm(self._alpha[cur_slice], self._Re[cur_slice], self._M[cur_slice])

                index += num_cps

        # Correct section moment coefficient for sweep
        if self._use_swept_sections:
            self._Cm = self._Cm*self._C_sweep_inv

        # Inviscid moment due to sectional properties
        if self._use_in_plane:
            dM_section = (self._redim_in_plane*self._c_bar*self._Cm)[:,np.newaxis]*self._u_s
        else:
            dM_section = (self._redim_full*self._c_bar*self._Cm)[:,np.newaxis]*self._u_s

        # Inviscid moment due to vortex lift and total inviscid moment
        dM_vortex = np.cross(self._r_CG, self._dF_inv)
        self._dM_inv = dM_vortex+dM_section

        # Determine viscous drag vector
        dD = self._redim_full*self._CD
        if self._use_total_velocity or self._match_machup_pro:
            self._dF_visc = dD[:,np.newaxis]*self._u_i
        else:
            self._dF_visc = dD[:,np.newaxis]*self._u_inf

        # Moment due to viscous drag
        self._dM_visc = np.cross(self._r_CG, self._dF_visc)

        # Loop through airplanes to gather necessary data
        index = 0
        for airplane_object in self._airplane_objects:
            airplane_name = airplane_object.name

            # Initialize totals
            if body_frame:
                FM_b_inv_airplane_total = np.zeros(6)
                FM_b_vis_airplane_total = np.zeros(6)
            if wind_frame:
                FM_w_inv_airplane_total = np.zeros(6)
                FM_w_vis_airplane_total = np.zeros(6)
            if stab_frame:
                FM_s_inv_airplane_total = np.zeros(6)
                FM_s_vis_airplane_total = np.zeros(6)

            # Initialize dictionary keys
            self._FM[airplane_name] = {
                "inviscid" : {},
                "viscous" : {},
                "total" : {}
            }
            if non_dimensional:
                self._FM[airplane_name]["inviscid"] = copy.deepcopy(empty_coef_dict)
                self._FM[airplane_name]["viscous"] = copy.deepcopy(empty_coef_dict)
            if dimensional:
                self._FM[airplane_name]["inviscid"].update(copy.deepcopy(empty_FM_dict))
                self._FM[airplane_name]["viscous"].update(copy.deepcopy(empty_FM_dict))

            # Determine reference freestream vector in body-fixed frame (used for resolving L, D, and S)
            v_inf = -airplane_object.v + self._get_wind(airplane_object.p_bar)
            V_inf = np.linalg.norm(v_inf)
            u_inf = quat_trans(airplane_object.q, (v_inf/V_inf).flatten())

            # Determine rotations to wind and stability frames
            if stab_frame or wind_frame:
                u_lift = np.cross(u_inf, [0.,1.,0.])
                u_lift = u_lift/np.linalg.norm(u_lift)
            if stab_frame:
                u_x_stab = np.cross(u_lift, [0.0, 1.0, 0.0])
                u_x_stab = u_x_stab/np.linalg.norm(u_x_stab)
                rot_to_stab = np.array([u_x_stab, [0.0, 1.0, 0.0], -u_lift])
            if wind_frame:
                u_side = np.cross(u_lift, u_inf)
                u_side = u_side/np.linalg.norm(u_side)
                rot_to_wind = np.array([u_inf, u_side, u_lift])

            # Determine reference parameters
            if non_dimensional:
                non_dim_inv = 2.0/(self._get_density(airplane_object.p_bar)*V_inf*V_inf*airplane_object.S_w)
                lat_non_dim_inv = non_dim_inv/airplane_object.l_ref_lat
                lon_non_dim_inv = non_dim_inv/airplane_object.l_ref_lon

            # Loop through segments
            for segment in airplane_object.segments:
                num_cps = segment.N
                segment_name = segment.name
                cur_slice = slice(index, index+num_cps)

                # Get drag coef and redimensionalize
                F_b_visc = quat_trans(airplane_object.q, np.sum(self._dF_visc[cur_slice], axis=0))

                # Determine viscous moment vector
                M_b_visc = quat_trans(airplane_object.q, np.sum(self._dM_visc[cur_slice], axis=0))

                # Determine inviscid force vector
                F_b_inv = quat_trans(airplane_object.q, np.sum(self._dF_inv[cur_slice], axis=0))

                # Determine inviscid moment vector
                M_b_inv = quat_trans(airplane_object.q, np.sum(self._dM_inv[cur_slice], axis=0))

                # Rotate frames
                if wind_frame:
                    F_w_visc = np.matmul(rot_to_wind, F_b_visc)
                    F_w_inv = np.matmul(rot_to_wind, F_b_inv)
                    M_w_visc = np.matmul(rot_to_wind, M_b_visc)
                    M_w_inv = np.matmul(rot_to_wind, M_b_inv)
                if stab_frame:
                    F_s_visc = np.matmul(rot_to_stab, F_b_visc)
                    F_s_inv = np.matmul(rot_to_stab, F_b_inv)
                    M_s_visc = np.matmul(rot_to_stab, M_b_visc)
                    M_s_inv = np.matmul(rot_to_stab, M_b_inv)

                # Store
                if report_by_segment:
                    if non_dimensional:
                        if body_frame:
                            self._FM[airplane_name]["viscous"]["Cx"][segment_name] = F_b_visc[0].item()*non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cy"][segment_name] = F_b_visc[1].item()*non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cz"][segment_name] = F_b_visc[2].item()*non_dim_inv

                            self._FM[airplane_name]["viscous"]["Cl"][segment_name] = M_b_visc[0].item()*lat_non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cm"][segment_name] = M_b_visc[1].item()*lon_non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cn"][segment_name] = M_b_visc[2].item()*lat_non_dim_inv

                            self._FM[airplane_name]["inviscid"]["Cx"][segment_name] = F_b_inv[0].item()*non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cy"][segment_name] = F_b_inv[1].item()*non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cz"][segment_name] = F_b_inv[2].item()*non_dim_inv

                            self._FM[airplane_name]["inviscid"]["Cl"][segment_name] = M_b_inv[0].item()*lat_non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cm"][segment_name] = M_b_inv[1].item()*lon_non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cn"][segment_name] = M_b_inv[2].item()*lat_non_dim_inv

                        if wind_frame:
                            self._FM[airplane_name]["viscous"]["CD"][segment_name] = F_w_visc[0].item()*non_dim_inv
                            self._FM[airplane_name]["viscous"]["CS"][segment_name] = F_w_visc[1].item()*non_dim_inv
                            self._FM[airplane_name]["viscous"]["CL"][segment_name] = F_w_visc[2].item()*non_dim_inv

                            self._FM[airplane_name]["viscous"]["Cl_w"][segment_name] = M_w_visc[0].item()*lat_non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cm_w"][segment_name] = M_w_visc[1].item()*lon_non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cn_w"][segment_name] = M_w_visc[2].item()*lat_non_dim_inv

                            self._FM[airplane_name]["inviscid"]["CD"][segment_name] = F_w_inv[0].item()*non_dim_inv
                            self._FM[airplane_name]["inviscid"]["CS"][segment_name] = F_w_inv[1].item()*non_dim_inv
                            self._FM[airplane_name]["inviscid"]["CL"][segment_name] = F_w_inv[2].item()*non_dim_inv

                            self._FM[airplane_name]["inviscid"]["Cl_w"][segment_name] = M_w_inv[0].item()*lat_non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cm_w"][segment_name] = M_w_inv[1].item()*lon_non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cn_w"][segment_name] = M_w_inv[2].item()*lat_non_dim_inv

                        if stab_frame:
                            self._FM[airplane_name]["viscous"]["Cx_s"][segment_name] = F_s_visc[0].item()*non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cy_s"][segment_name] = F_s_visc[1].item()*non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cz_s"][segment_name] = F_s_visc[2].item()*non_dim_inv

                            self._FM[airplane_name]["viscous"]["Cl_s"][segment_name] = M_s_visc[0].item()*lat_non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cm_s"][segment_name] = M_s_visc[1].item()*lon_non_dim_inv
                            self._FM[airplane_name]["viscous"]["Cn_s"][segment_name] = M_s_visc[2].item()*lat_non_dim_inv

                            self._FM[airplane_name]["inviscid"]["Cx_s"][segment_name] = F_s_inv[0].item()*non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cy_s"][segment_name] = F_s_inv[1].item()*non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cz_s"][segment_name] = F_s_inv[2].item()*non_dim_inv

                            self._FM[airplane_name]["inviscid"]["Cl_s"][segment_name] = M_s_inv[0].item()*lat_non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cm_s"][segment_name] = M_s_inv[1].item()*lon_non_dim_inv
                            self._FM[airplane_name]["inviscid"]["Cn_s"][segment_name] = M_s_inv[2].item()*lat_non_dim_inv

                    if dimensional:
                        if body_frame:
                            self._FM[airplane_name]["viscous"]["Fx"][segment_name] = F_b_visc[0].item()
                            self._FM[airplane_name]["viscous"]["Fy"][segment_name] = F_b_visc[1].item()
                            self._FM[airplane_name]["viscous"]["Fz"][segment_name] = F_b_visc[2].item()

                            self._FM[airplane_name]["viscous"]["Mx"][segment_name] = M_b_visc[0].item()
                            self._FM[airplane_name]["viscous"]["My"][segment_name] = M_b_visc[1].item()
                            self._FM[airplane_name]["viscous"]["Mz"][segment_name] = M_b_visc[2].item()

                            self._FM[airplane_name]["inviscid"]["Fx"][segment_name] = F_b_inv[0].item()
                            self._FM[airplane_name]["inviscid"]["Fy"][segment_name] = F_b_inv[1].item()
                            self._FM[airplane_name]["inviscid"]["Fz"][segment_name] = F_b_inv[2].item()

                            self._FM[airplane_name]["inviscid"]["Mx"][segment_name] = M_b_inv[0].item()
                            self._FM[airplane_name]["inviscid"]["My"][segment_name] = M_b_inv[1].item()
                            self._FM[airplane_name]["inviscid"]["Mz"][segment_name] = M_b_inv[2].item()

                        if wind_frame:
                            self._FM[airplane_name]["viscous"]["FD"][segment_name] = F_w_visc[0].item()
                            self._FM[airplane_name]["viscous"]["FS"][segment_name] = F_w_visc[1].item()
                            self._FM[airplane_name]["viscous"]["FL"][segment_name] = F_w_visc[2].item()

                            self._FM[airplane_name]["inviscid"]["FD"][segment_name] = F_w_inv[0].item()
                            self._FM[airplane_name]["inviscid"]["FS"][segment_name] = F_w_inv[1].item()
                            self._FM[airplane_name]["inviscid"]["FL"][segment_name] = F_w_inv[2].item()

                            self._FM[airplane_name]["viscous"]["Mx_w"][segment_name] = M_w_visc[0].item()
                            self._FM[airplane_name]["viscous"]["My_w"][segment_name] = M_w_visc[1].item()
                            self._FM[airplane_name]["viscous"]["Mz_w"][segment_name] = M_w_visc[2].item()

                            self._FM[airplane_name]["inviscid"]["Mx_w"][segment_name] = M_w_inv[0].item()
                            self._FM[airplane_name]["inviscid"]["My_w"][segment_name] = M_w_inv[1].item()
                            self._FM[airplane_name]["inviscid"]["Mz_w"][segment_name] = M_w_inv[2].item()

                        if stab_frame:
                            self._FM[airplane_name]["viscous"]["Fx_s"][segment_name] = F_s_visc[0].item()
                            self._FM[airplane_name]["viscous"]["Fy_s"][segment_name] = F_s_visc[1].item()
                            self._FM[airplane_name]["viscous"]["Fz_s"][segment_name] = F_s_visc[2].item()

                            self._FM[airplane_name]["viscous"]["Mx_s"][segment_name] = M_s_visc[0].item()
                            self._FM[airplane_name]["viscous"]["My_s"][segment_name] = M_s_visc[1].item()
                            self._FM[airplane_name]["viscous"]["Mz_s"][segment_name] = M_s_visc[2].item()

                            self._FM[airplane_name]["inviscid"]["Fx_s"][segment_name] = F_s_inv[0].item()
                            self._FM[airplane_name]["inviscid"]["Fy_s"][segment_name] = F_s_inv[1].item()
                            self._FM[airplane_name]["inviscid"]["Fz_s"][segment_name] = F_s_inv[2].item()

                            self._FM[airplane_name]["inviscid"]["Mx_s"][segment_name] = M_s_inv[0].item()
                            self._FM[airplane_name]["inviscid"]["My_s"][segment_name] = M_s_inv[1].item()
                            self._FM[airplane_name]["inviscid"]["Mz_s"][segment_name] = M_s_inv[2].item()

                # Sum up totals
                if body_frame:
                    FM_b_inv_airplane_total[:3] += F_b_inv
                    FM_b_inv_airplane_total[3:] += M_b_inv
                    FM_b_vis_airplane_total[:3] += F_b_visc
                    FM_b_vis_airplane_total[3:] += M_b_visc
                if wind_frame:
                    FM_w_inv_airplane_total[:3] += F_w_inv
                    FM_w_inv_airplane_total[3:] += M_w_inv
                    FM_w_vis_airplane_total[:3] += F_w_visc
                    FM_w_vis_airplane_total[3:] += M_w_visc
                if stab_frame:
                    FM_s_inv_airplane_total[:3] += F_s_inv
                    FM_s_inv_airplane_total[3:] += M_s_inv
                    FM_s_vis_airplane_total[:3] += F_s_visc
                    FM_s_vis_airplane_total[3:] += M_s_visc

                index += num_cps

            if non_dimensional:
                if body_frame:
                    self._FM[airplane_name]["inviscid"]["Cx"]["total"] = FM_b_inv_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cy"]["total"] = FM_b_inv_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cz"]["total"] = FM_b_inv_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cl"]["total"] = FM_b_inv_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cm"]["total"] = FM_b_inv_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cn"]["total"] = FM_b_inv_airplane_total[5].item()*lat_non_dim_inv

                    # Store the total viscous force and moment
                    self._FM[airplane_name]["viscous"]["Cx"]["total"] = FM_b_vis_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cy"]["total"] = FM_b_vis_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cz"]["total"] = FM_b_vis_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cl"]["total"] = FM_b_vis_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cm"]["total"] = FM_b_vis_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cn"]["total"] = FM_b_vis_airplane_total[5].item()*lat_non_dim_inv

                    # Determine total force and moment for the airplane
                    FM_b_airplane_total = FM_b_vis_airplane_total+FM_b_inv_airplane_total
                    self._FM[airplane_name]["total"]["Cx"] = FM_b_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["Cy"] = FM_b_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["Cz"] = FM_b_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["Cl"] = FM_b_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["total"]["Cm"] = FM_b_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["total"]["Cn"] = FM_b_airplane_total[5].item()*lat_non_dim_inv

                if stab_frame:
                    self._FM[airplane_name]["inviscid"]["Cx_s"]["total"] = FM_s_inv_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cy_s"]["total"] = FM_s_inv_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cz_s"]["total"] = FM_s_inv_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cl_s"]["total"] = FM_s_inv_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cm_s"]["total"] = FM_s_inv_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cn_s"]["total"] = FM_s_inv_airplane_total[5].item()*lat_non_dim_inv

                    # Store the total viscous force and moment
                    self._FM[airplane_name]["viscous"]["Cx_s"]["total"] = FM_s_vis_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cy_s"]["total"] = FM_s_vis_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cz_s"]["total"] = FM_s_vis_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cl_s"]["total"] = FM_s_vis_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cm_s"]["total"] = FM_s_vis_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cn_s"]["total"] = FM_s_vis_airplane_total[5].item()*lat_non_dim_inv

                    # Determine total force and moment for the airplane
                    FM_s_airplane_total = FM_s_vis_airplane_total+FM_s_inv_airplane_total
                    self._FM[airplane_name]["total"]["Cx_s"] = FM_s_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["Cy_s"] = FM_s_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["Cz_s"] = FM_s_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["Cl_s"] = FM_s_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["total"]["Cm_s"] = FM_s_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["total"]["Cn_s"] = FM_s_airplane_total[5].item()*lat_non_dim_inv

                if wind_frame:
                    self._FM[airplane_name]["inviscid"]["CD"]["total"] = FM_w_inv_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["CS"]["total"] = FM_w_inv_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["CL"]["total"] = FM_w_inv_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cl_w"]["total"] = FM_w_inv_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cm_w"]["total"] = FM_w_inv_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["inviscid"]["Cn_w"]["total"] = FM_w_inv_airplane_total[5].item()*lat_non_dim_inv

                    # Store the total viscous force and moment
                    self._FM[airplane_name]["viscous"]["CD"]["total"] = FM_w_vis_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["CS"]["total"] = FM_w_vis_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["CL"]["total"] = FM_w_vis_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cl_w"]["total"] = FM_w_vis_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cm_w"]["total"] = FM_w_vis_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["viscous"]["Cn_w"]["total"] = FM_w_vis_airplane_total[5].item()*lat_non_dim_inv

                    # Determine total force and moment for the airplane
                    FM_w_airplane_total = FM_w_vis_airplane_total+FM_w_inv_airplane_total
                    self._FM[airplane_name]["total"]["CD"] = FM_w_airplane_total[0].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["CS"] = FM_w_airplane_total[1].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["CL"] = FM_w_airplane_total[2].item()*non_dim_inv
                    self._FM[airplane_name]["total"]["Cl_w"] = FM_w_airplane_total[3].item()*lat_non_dim_inv
                    self._FM[airplane_name]["total"]["Cm_w"] = FM_w_airplane_total[4].item()*lon_non_dim_inv
                    self._FM[airplane_name]["total"]["Cn_w"] = FM_w_airplane_total[5].item()*lat_non_dim_inv

            if dimensional:
                if body_frame:
                    self._FM[airplane_name]["inviscid"]["Fx"]["total"] = FM_b_inv_airplane_total[0].item()
                    self._FM[airplane_name]["inviscid"]["Fy"]["total"] = FM_b_inv_airplane_total[1].item()
                    self._FM[airplane_name]["inviscid"]["Fz"]["total"] = FM_b_inv_airplane_total[2].item()
                    self._FM[airplane_name]["inviscid"]["Mx"]["total"] = FM_b_inv_airplane_total[3].item()
                    self._FM[airplane_name]["inviscid"]["My"]["total"] = FM_b_inv_airplane_total[4].item()
                    self._FM[airplane_name]["inviscid"]["Mz"]["total"] = FM_b_inv_airplane_total[5].item()

                    # Store the total viscous force and moment
                    self._FM[airplane_name]["viscous"]["Fx"]["total"] = FM_b_vis_airplane_total[0].item()
                    self._FM[airplane_name]["viscous"]["Fy"]["total"] = FM_b_vis_airplane_total[1].item()
                    self._FM[airplane_name]["viscous"]["Fz"]["total"] = FM_b_vis_airplane_total[2].item()
                    self._FM[airplane_name]["viscous"]["Mx"]["total"] = FM_b_vis_airplane_total[3].item()
                    self._FM[airplane_name]["viscous"]["My"]["total"] = FM_b_vis_airplane_total[4].item()
                    self._FM[airplane_name]["viscous"]["Mz"]["total"] = FM_b_vis_airplane_total[5].item()

                    # Determine total force and moment for the airplane
                    FM_b_airplane_total = FM_b_vis_airplane_total+FM_b_inv_airplane_total
                    self._FM[airplane_name]["total"]["Fx"] = FM_b_airplane_total[0].item()
                    self._FM[airplane_name]["total"]["Fy"] = FM_b_airplane_total[1].item()
                    self._FM[airplane_name]["total"]["Fz"] = FM_b_airplane_total[2].item()
                    self._FM[airplane_name]["total"]["Mx"] = FM_b_airplane_total[3].item()
                    self._FM[airplane_name]["total"]["My"] = FM_b_airplane_total[4].item()
                    self._FM[airplane_name]["total"]["Mz"] = FM_b_airplane_total[5].item()

                if stab_frame:
                    self._FM[airplane_name]["inviscid"]["Fx_s"]["total"] = FM_s_inv_airplane_total[0].item()
                    self._FM[airplane_name]["inviscid"]["Fy_s"]["total"] = FM_s_inv_airplane_total[1].item()
                    self._FM[airplane_name]["inviscid"]["Fz_s"]["total"] = FM_s_inv_airplane_total[2].item()
                    self._FM[airplane_name]["inviscid"]["Mx_s"]["total"] = FM_s_inv_airplane_total[3].item()
                    self._FM[airplane_name]["inviscid"]["My_s"]["total"] = FM_s_inv_airplane_total[4].item()
                    self._FM[airplane_name]["inviscid"]["Mz_s"]["total"] = FM_s_inv_airplane_total[5].item()

                    # Store the total viscous force and moment
                    self._FM[airplane_name]["viscous"]["Fx_s"]["total"] = FM_s_vis_airplane_total[0].item()
                    self._FM[airplane_name]["viscous"]["Fy_s"]["total"] = FM_s_vis_airplane_total[1].item()
                    self._FM[airplane_name]["viscous"]["Fz_s"]["total"] = FM_s_vis_airplane_total[2].item()
                    self._FM[airplane_name]["viscous"]["Mx_s"]["total"] = FM_s_vis_airplane_total[3].item()
                    self._FM[airplane_name]["viscous"]["My_s"]["total"] = FM_s_vis_airplane_total[4].item()
                    self._FM[airplane_name]["viscous"]["Mz_s"]["total"] = FM_s_vis_airplane_total[5].item()

                    # Determine total force and moment for the airplane
                    FM_s_airplane_total = FM_s_vis_airplane_total+FM_s_inv_airplane_total
                    self._FM[airplane_name]["total"]["Fx_s"] = FM_s_airplane_total[0].item()
                    self._FM[airplane_name]["total"]["Fy_s"] = FM_s_airplane_total[1].item()
                    self._FM[airplane_name]["total"]["Fz_s"] = FM_s_airplane_total[2].item()
                    self._FM[airplane_name]["total"]["Mx_s"] = FM_s_airplane_total[3].item()
                    self._FM[airplane_name]["total"]["My_s"] = FM_s_airplane_total[4].item()
                    self._FM[airplane_name]["total"]["Mz_s"] = FM_s_airplane_total[5].item()

                if wind_frame:
                    self._FM[airplane_name]["inviscid"]["FD"]["total"] = FM_w_inv_airplane_total[0].item()
                    self._FM[airplane_name]["inviscid"]["FS"]["total"] = FM_w_inv_airplane_total[1].item()
                    self._FM[airplane_name]["inviscid"]["FL"]["total"] = FM_w_inv_airplane_total[2].item()
                    self._FM[airplane_name]["inviscid"]["Mx_w"]["total"] = FM_w_inv_airplane_total[3].item()
                    self._FM[airplane_name]["inviscid"]["My_w"]["total"] = FM_w_inv_airplane_total[4].item()
                    self._FM[airplane_name]["inviscid"]["Mz_w"]["total"] = FM_w_inv_airplane_total[5].item()

                    # Store the total viscous force and moment
                    self._FM[airplane_name]["viscous"]["FD"]["total"] = FM_w_vis_airplane_total[0].item()
                    self._FM[airplane_name]["viscous"]["FS"]["total"] = FM_w_vis_airplane_total[1].item()
                    self._FM[airplane_name]["viscous"]["FL"]["total"] = FM_w_vis_airplane_total[2].item()
                    self._FM[airplane_name]["viscous"]["Mx_w"]["total"] = FM_w_vis_airplane_total[3].item()
                    self._FM[airplane_name]["viscous"]["My_w"]["total"] = FM_w_vis_airplane_total[4].item()
                    self._FM[airplane_name]["viscous"]["Mz_w"]["total"] = FM_w_vis_airplane_total[5].item()

                    # Determine total force and moment for the airplane
                    FM_w_airplane_total = FM_w_vis_airplane_total+FM_w_inv_airplane_total
                    self._FM[airplane_name]["total"]["FD"] = FM_w_airplane_total[0].item()
                    self._FM[airplane_name]["total"]["FS"] = FM_w_airplane_total[1].item()
                    self._FM[airplane_name]["total"]["FL"] = FM_w_airplane_total[2].item()
                    self._FM[airplane_name]["total"]["Mx_w"] = FM_w_airplane_total[3].item()
                    self._FM[airplane_name]["total"]["My_w"] = FM_w_airplane_total[4].item()
                    self._FM[airplane_name]["total"]["Mz_w"] = FM_w_airplane_total[5].item()

        return time.time()-start_time


    def solve_forces(self, **kwargs):
        """Solves the NLL equations to determine the forces and moments on each aircraft.

        Parameters
        ----------
        filename : str
            File to export the force and moment results to. Should be .json. If not specified, 
            results will not be exported to a file.

        non_dimensional : bool
            If this is set to True, nondimensional coefficients will be included in the results.
            Defaults to True.

        dimensional : bool
            If this is set to True, dimensional forces and moments will be included in the results.
            Defaults to True.

        report_by_segment : bool
            Whether to include results broken down by wing segment. Defaults to False.

        body_frame : boolean, optional
            Whether to output results in the body-fixed frame. Defaults to True.

        stab_frame : boolean, optional
            Whether to output results in the stability frame. Defaults to False.

        wind_frame : boolean, optional
            Whether to output results in the wind frame. Defaults to True.

        verbose : bool
            Whether to display timing and convergence information. Defaults to False.

        Returns
        -------
        FM : dict
            Dictionary of forces and moments acting on each wing segment.
        """

        # Check for aircraft
        if self._num_aircraft == 0:
            raise RuntimeError("There are no aircraft in this scene. No calculations can be performed.")

        # Initialize timing and error handling
        self._FM = {}
        fsolve_time = 0.0
        linear_time = 0.0
        nonlinear_time = 0.0
        integrate_time = 0.0

        try:

            # Solve for gamma distribution using fsolve
            if self._solver_type == "scipy_fsolve":
                fsolve_time = self._solve_w_scipy(**kwargs)

            # Solve for gamma using analytical solvers
            if self._solver_type != "scipy_fsolve" or fsolve_time == -1:

                # Linear solution
                linear_time = self._solve_linear(**kwargs)

                # Nonlinear improvement
                if self._solver_type == "nonlinear" or fsolve_time == -1:
                    try:
                        nonlinear_time = self._solve_nonlinear(**kwargs, scipy_failed=(fsolve_time==-1))
                    except KeyboardInterrupt:
                        print("")
                        print("!!!Nonlinear solver interrupted by Ctrl+C event. Moving on to force and moment integration...")
                        nonlinear_time = time.time()-self._nonlinear_start_time

                if fsolve_time == -1:
                    fsolve_time = 0.0

        except Exception as e:
            self._handle_error(e)

        try:

            # Integrate forces and moments
            integrate_time = self._integrate_forces_and_moments(**kwargs)

        except Exception as e:
            self._handle_error(e)

        # Output timing
        verbose = kwargs.get("verbose", False)
        if verbose:
            print("Time to compute circulation distribution using scipy.fsolve: {0} s".format(fsolve_time))
            print("Time to compute circulation distribution using linear equations: {0} s".format(linear_time))
            print("Time to compute nonlinear improvement to circulation distribution: {0} s".format(nonlinear_time))
            total_time = linear_time+nonlinear_time+integrate_time+fsolve_time
            print("Time to integrate forces: {0} s".format(integrate_time))
            print("Total time: {0} s".format(total_time))
            try:
                print("Solution rate: {0} Hz".format(1/total_time))
            except ZeroDivisionError:
                pass

        # Output to file
        filename = kwargs.get("filename", None)
        if filename is not None:
            with open(filename, 'w') as json_file_handle:
                json.dump(self._FM, json_file_handle, indent=4)

        # Let certain functions know the results are now available
        self._solved = True

        return self._FM


    def set_aircraft_state(self, state={}, aircraft=None):
        """Sets the state of the given aircraft.

        Parameters
        ----------
        state : dict
            Dictionary describing the state as specified in 
            'Creating Input Files for MachUp'. Any values not
            given default to their original defaults. The
            previous state of the aircraft is in no way preserved.

        aircraft : str
            The name of the aircraft to set the state of. If there
            is only one aircraft in the scene, this does not need 
            to be specified.
        """

        # Specify the only aircraft if not already specified
        if aircraft is None:
            if self._num_aircraft == 1:
                aircraft = list(self._airplanes.keys())[0]
            else:
                raise IOError("Aircraft name must be specified if there is more than one aircraft in the scene.")

        # Determine wind velocity
        aircraft_position = np.array(state.get("position", [0.0, 0.0, 0.0]))
        v_wind = self._get_wind(aircraft_position)

        # Set state
        old_position = self._airplanes[aircraft].p_bar
        old_orient = self._airplanes[aircraft].q
        self._airplanes[aircraft].set_state(**state, v_wind=v_wind)
        aircraft_orient = self._airplanes[aircraft].q

        # If the position has changed, then we need to update the geometry
        if not np.allclose(old_position, aircraft_position) or not np.allclose(old_orient, aircraft_orient):
            self._perform_geometry_and_atmos_calcs()


    def set_aircraft_control_state(self, control_state={}, aircraft=None):
        """Sets the control state of the given aircraft.

        Parameters
        ----------
        control_state : dict
            Dictionary describing the control state. Each key value pair should be
            the name of the control and its deflection in degrees.

        aircraft : str
            The name of the aircraft to set the state of. If there
            is only one aircraft in the scene, this does not need 
            to be specified.
        """

        # Specify the only aircraft if not already specified
        if aircraft is None:
            if self._num_aircraft == 1:
                aircraft = list(self._airplanes.keys())[0]
            else:
                raise IOError("Aircraft name must be specified if there is more than one aircraft in the scene.")

        # Set state
        self._airplanes[aircraft].set_control_state(control_state)
        self._solved = False


    def display_wireframe(self, **kwargs):
        """Displays a 3D wireframe plot of the scene.

        Parameters
        ----------
        show_vortices : bool, optional
            If this is set to True, the distribution of horseshoe vortices along each lifting surface will be 
            shown. Defaults to True.

        show_legend : bool, optional
            If this is set to True, a legend will appear detailing which color corresponds to which wing segment.
            Otherwise, the wing segments are all black. Defaults to False.

        filename : str, optional
            File to save an image of the wireframe to. If specified, the wireframe will not be 
            automatically displayed. If not specified, the wireframe will display to the user 
            and not save.
        """

        # Setup 3D figure
        fig = plt.figure(figsize=plt.figaspect(1.0))
        ax = fig.gca(projection='3d')

        # This matters for setting up the plot axis limits
        first_segment = True

        # Kwargs
        show_vortices = kwargs.get("show_vortices", True)
        show_legend = kwargs.get("show_legend", False)
        filename = kwargs.get("filename", None)

        # If the user wants the vortices displayed, make sure we've set the flow properties
        if show_vortices and not self._solved:
            self._calc_invariant_flow_properties()

        # Loop through airplanes
        for airplane_name, airplane_object in self._airplanes.items():

            # Loop through segments
            for segment_name, segment_object in airplane_object.wing_segments.items():

                # Get the outline points and transform to earth-fixed
                points, cntrl_points = segment_object.get_outline_points()
                points = airplane_object.p_bar+quat_inv_trans(airplane_object.q, points)

                # Plot control surfaces
                if cntrl_points is not None:
                    cntrl_points = airplane_object.p_bar+quat_inv_trans(airplane_object.q, cntrl_points)
                    ax.plot(cntrl_points[:,0], cntrl_points[:,1], cntrl_points[:,2], 'k-')

                # Decide if colors matter and the segment names need to be stored
                if show_legend:
                    ax.plot(points[:,0], points[:,1], points[:,2], '-', label=airplane_name+segment_name)
                else:
                    ax.plot(points[:,0], points[:,1], points[:,2], 'k-')

                # Figure out if the segment just added increases any needed axis limits
                if first_segment:
                    x_lims = [min(points[:,0].flatten()), max(points[:,0].flatten())]
                    y_lims = [min(points[:,1].flatten()), max(points[:,1].flatten())]
                    z_lims = [min(points[:,2].flatten()), max(points[:,2].flatten())]
                    first_segment = False
                else:
                    x_lims = [min(x_lims[0], min(points[:,0].flatten())), max(x_lims[1], max(points[:,0].flatten()))]
                    y_lims = [min(y_lims[0], min(points[:,1].flatten())), max(y_lims[1], max(points[:,1].flatten()))]
                    z_lims = [min(z_lims[0], min(points[:,2].flatten())), max(z_lims[1], max(points[:,2].flatten()))]

            # Add vortices
            if show_vortices:
                q = airplane_object.q

                # Loop through wings
                for wing_slice in airplane_object.wing_slices:

                    # Declare storage
                    wing_N = wing_slice.stop-wing_slice.start
                    vortex_points = np.zeros((wing_N*6,3))
                    
                    # Gather and arrange node locations
                    vortex_points[0:wing_N*6+0:6,:] = quat_inv_trans(q, airplane_object.P0_joint[wing_slice])+self._P0_joint_u_inf[wing_slice]*2*airplane_object.l_ref_lon
                    vortex_points[1:wing_N*6+1:6,:] = quat_inv_trans(q, airplane_object.P0_joint[wing_slice])
                    vortex_points[2:wing_N*6+2:6,:] = quat_inv_trans(q, airplane_object.P0[wing_slice])
                    vortex_points[3:wing_N*6+3:6,:] = quat_inv_trans(q, airplane_object.P1[wing_slice])
                    vortex_points[4:wing_N*6+4:6,:] = quat_inv_trans(q, airplane_object.P1_joint[wing_slice])
                    vortex_points[5:wing_N*6+5:6,:] = quat_inv_trans(q, airplane_object.P1_joint[wing_slice])+self._P1_joint_u_inf[wing_slice]*2*airplane_object.l_ref_lon

                    # Add to plot
                    ax.plot(vortex_points[:,0], vortex_points[:,1], vortex_points[:,2], 'b--')

        # Add legend
        if show_legend:
            ax.legend()

        # Set axis labels
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        # Find out which axis has the widest limits
        x_diff = x_lims[1]-x_lims[0]
        y_diff = y_lims[1]-y_lims[0]
        z_diff = z_lims[1]-z_lims[0]
        max_diff = max([x_diff, y_diff, z_diff])

        # Determine the center of each set of axis limits
        x_cent = x_lims[0]+0.5*x_diff
        y_cent = y_lims[0]+0.5*y_diff
        z_cent = z_lims[0]+0.5*z_diff

        # Scale the axis limits so they all have the same width as the widest set
        x_lims[0] = x_cent-0.5*max_diff
        x_lims[1] = x_cent+0.5*max_diff

        y_lims[0] = y_cent-0.5*max_diff
        y_lims[1] = y_cent+0.5*max_diff

        z_lims[0] = z_cent-0.5*max_diff
        z_lims[1] = z_cent+0.5*max_diff

        # Set limits so it is a right-handed coordinate system with z pointing down
        ax.set_xlim3d(x_lims[1], x_lims[0])
        ax.set_ylim3d(y_lims[0], y_lims[1])
        ax.set_zlim3d(z_lims[1], z_lims[0])

        # Output figure
        if filename is not None:
            plt.savefig(filename)
            plt.close()
        else:
            plt.show()


    def derivatives(self, **kwargs):
        """Determines the stability, damping, and control derivatives at the 
        current state. Uses a central difference scheme. Note that the angular
        rates for the damping derivatives will be in the frame the angular
        rates were originally given in.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the aerodynamic derivatives 
            of. Defaults to all aircraft in the scene.

        filename : str
            File to export the results to. Defaults to no file.

        body_frame : boolean, optional
            Whether to output results in the body-fixed frame. Defaults to True.

        stab_frame : boolean, optional
            Whether to output results in the stability frame. Defaults to False.

        wind_frame : boolean, optional
            Whether to output results in the wind frame. Defaults to True.

        Returns
        -------
        dict
            A dictionary of stability, damping, and control derivatives.
        """
        derivs = {}

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}
            # Determine stability derivatives
            derivs[aircraft_name]["stability"] = self.stability_derivatives(aircraft=aircraft_name, **kwargs)[aircraft_name]
        
            # Determine damping derivatives
            derivs[aircraft_name]["damping"] = self.damping_derivatives(aircraft=aircraft_name, **kwargs)[aircraft_name]

            # Determine control derivatives
            derivs[aircraft_name]["control"] = self.control_derivatives(aircraft=aircraft_name, **kwargs)[aircraft_name]

        # Export to file
        filename = kwargs.get("filename", None)
        if filename is not None:
            with open(filename, 'w') as output_handle:
                json.dump(derivs, output_handle, indent=4)

        return derivs


    def stability_derivatives(self, dtheta=0.5, **kwargs):
        """Determines the stability derivatives at the current state. Uses 
        a central difference scheme.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the stability derivatives 
            of. Defaults to all aircraft in the scene.

        dtheta : float
            The finite difference in degrees used to perturb alpha and beta
            and determine the derivatives. Defaults to 0.5

        body_frame : boolean, optional
            Whether to output results in the body-fixed frame. Defaults to True.

        stab_frame : boolean, optional
            Whether to output results in the stability frame. Defaults to False.

        wind_frame : boolean, optional
            Whether to output results in the wind frame. Defaults to True.

        Returns
        -------
        dict
            A dictionary of stability derivatives.
        """
        derivs= {}

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Determine output frames
        body_frame, stab_frame, wind_frame = self._get_frames(**kwargs)

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}
            # Get current aerodynamic state
            alpha_0, beta_0,_ = self._airplanes[aircraft_name].get_aerodynamic_state()

            # Perturb forward in alpha
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0+dtheta)
            self.solve_forces(dimensional=False, **kwargs)
            FM_dalpha_fwd = self._FM

            # Perturb backward in alpha
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0-dtheta)
            self.solve_forces(dimensional=False, **kwargs)
            FM_dalpha_bwd = self._FM

            # Perturb forward in beta
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0, beta=beta_0+dtheta) # We have to reset alpha on this one
            self.solve_forces(dimensional=False, **kwargs)
            FM_dbeta_fwd = self._FM

            # Perturb backward in beta
            self._airplanes[aircraft_name].set_aerodynamic_state(beta=beta_0-dtheta)
            self.solve_forces(dimensional=False, **kwargs)
            FM_dbeta_bwd = self._FM

            diff = 1/(2*np.radians(dtheta)) # The derivative is in radians

            if body_frame:
                derivs[aircraft_name]["Cx,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cx"]-FM_dalpha_bwd[aircraft_name]["total"]["Cx"])*diff
                derivs[aircraft_name]["Cy,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cy"]-FM_dalpha_bwd[aircraft_name]["total"]["Cy"])*diff
                derivs[aircraft_name]["Cz,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cz"]-FM_dalpha_bwd[aircraft_name]["total"]["Cz"])*diff
                derivs[aircraft_name]["Cl,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cl"]-FM_dalpha_bwd[aircraft_name]["total"]["Cl"])*diff
                derivs[aircraft_name]["Cm,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cm"]-FM_dalpha_bwd[aircraft_name]["total"]["Cm"])*diff
                derivs[aircraft_name]["Cn,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cn"]-FM_dalpha_bwd[aircraft_name]["total"]["Cn"])*diff

                derivs[aircraft_name]["Cx,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cx"]-FM_dbeta_bwd[aircraft_name]["total"]["Cx"])*diff
                derivs[aircraft_name]["Cy,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cy"]-FM_dbeta_bwd[aircraft_name]["total"]["Cy"])*diff
                derivs[aircraft_name]["Cz,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cz"]-FM_dbeta_bwd[aircraft_name]["total"]["Cz"])*diff
                derivs[aircraft_name]["Cl,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cl"]-FM_dbeta_bwd[aircraft_name]["total"]["Cl"])*diff
                derivs[aircraft_name]["Cm,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cm"]-FM_dbeta_bwd[aircraft_name]["total"]["Cm"])*diff
                derivs[aircraft_name]["Cn,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cn"]-FM_dbeta_bwd[aircraft_name]["total"]["Cn"])*diff

            if stab_frame:
                derivs[aircraft_name]["Cx_s,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cx_s"]-FM_dalpha_bwd[aircraft_name]["total"]["Cx_s"])*diff
                derivs[aircraft_name]["Cy_s,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cy_s"]-FM_dalpha_bwd[aircraft_name]["total"]["Cy_s"])*diff
                derivs[aircraft_name]["Cz_s,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cz_s"]-FM_dalpha_bwd[aircraft_name]["total"]["Cz_s"])*diff
                derivs[aircraft_name]["Cl_s,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cl_s"]-FM_dalpha_bwd[aircraft_name]["total"]["Cl_s"])*diff
                derivs[aircraft_name]["Cm_s,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cm_s"]-FM_dalpha_bwd[aircraft_name]["total"]["Cm_s"])*diff
                derivs[aircraft_name]["Cn_s,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cn_s"]-FM_dalpha_bwd[aircraft_name]["total"]["Cn_s"])*diff

                derivs[aircraft_name]["Cx_s,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cx_s"]-FM_dbeta_bwd[aircraft_name]["total"]["Cx_s"])*diff
                derivs[aircraft_name]["Cy_s,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cy_s"]-FM_dbeta_bwd[aircraft_name]["total"]["Cy_s"])*diff
                derivs[aircraft_name]["Cz_s,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cz_s"]-FM_dbeta_bwd[aircraft_name]["total"]["Cz_s"])*diff
                derivs[aircraft_name]["Cl_s,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cl_s"]-FM_dbeta_bwd[aircraft_name]["total"]["Cl_s"])*diff
                derivs[aircraft_name]["Cm_s,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cm_s"]-FM_dbeta_bwd[aircraft_name]["total"]["Cm_s"])*diff
                derivs[aircraft_name]["Cn_s,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cn_s"]-FM_dbeta_bwd[aircraft_name]["total"]["Cn_s"])*diff

            if wind_frame:
                derivs[aircraft_name]["CL,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CL"]-FM_dalpha_bwd[aircraft_name]["total"]["CL"])*diff
                derivs[aircraft_name]["CD,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CD"]-FM_dalpha_bwd[aircraft_name]["total"]["CD"])*diff
                derivs[aircraft_name]["CS,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CS"]-FM_dalpha_bwd[aircraft_name]["total"]["CS"])*diff
                derivs[aircraft_name]["Cl_w,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cl_w"]-FM_dalpha_bwd[aircraft_name]["total"]["Cl_w"])*diff
                derivs[aircraft_name]["Cm_w,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cm_w"]-FM_dalpha_bwd[aircraft_name]["total"]["Cm_w"])*diff
                derivs[aircraft_name]["Cn_w,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cn_w"]-FM_dalpha_bwd[aircraft_name]["total"]["Cn_w"])*diff

                derivs[aircraft_name]["CL,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["CL"]-FM_dbeta_bwd[aircraft_name]["total"]["CL"])*diff
                derivs[aircraft_name]["CD,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["CD"]-FM_dbeta_bwd[aircraft_name]["total"]["CD"])*diff
                derivs[aircraft_name]["CS,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["CS"]-FM_dbeta_bwd[aircraft_name]["total"]["CS"])*diff
                derivs[aircraft_name]["Cl_w,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cl_w"]-FM_dbeta_bwd[aircraft_name]["total"]["Cl_w"])*diff
                derivs[aircraft_name]["Cm_w,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cm_w"]-FM_dbeta_bwd[aircraft_name]["total"]["Cm_w"])*diff
                derivs[aircraft_name]["Cn_w,b"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cn_w"]-FM_dbeta_bwd[aircraft_name]["total"]["Cn_w"])*diff

                # Calculate static margin
                derivs[aircraft_name]["%_static_margin"] = -derivs[aircraft_name]["Cm_w,a"]/derivs[aircraft_name]["CL,a"]*100.0
        
            # Reset aerodynamic state
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0, beta=beta_0)
            self._solved = False

        return derivs


    def damping_derivatives(self, aircraft=None, dtheta_dot=0.005, **kwargs):
        """Determines the damping derivatives at the current state. Uses 
        a central difference scheme. Note, the damping derivatives are non-
        dimensionalized with respect to 2V/l_ref_lat and 2V/l_ref_lon. Also,
        the angular rates for the damping derivatives will be in the frame
        the angular rates were originally given in.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the damping derivatives 
            of. Defaults to all aircraft in the scene.

        dtheta_dot : float
            The finite difference used to perturb the angular rates of the aircraft
            and determine the derivatives. Given in radians per second. Defaults to 0.005.

        body_frame : boolean, optional
            Whether to output results in the body-fixed frame. Defaults to True.

        stab_frame : boolean, optional
            Whether to output results in the stability frame. Defaults to False.

        wind_frame : boolean, optional
            Whether to output results in the wind frame. Defaults to True.

        Returns
        -------
        dict
            A dictionary of damping derivatives.
        """
        derivs = {}

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Determine output frames
        body_frame, stab_frame, wind_frame = self._get_frames(**kwargs)

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}
            aircraft_object = self._airplanes[aircraft_name]

            # Get current aerodynamic state
            _,_,vel_0 = aircraft_object.get_aerodynamic_state()

            # Determine current angular rates and the frame they were specified in
            omega_0 = aircraft_object.w
            frame = aircraft_object.angular_rate_frame

            # Determine preturbations
            p_pert = np.array([dtheta_dot, 0.0, 0.0])
            q_pert = np.array([0.0, dtheta_dot, 0.0])
            r_pert = np.array([0.0, 0.0, dtheta_dot])

            if frame == "stab":
                p_pert = quat_inv_trans(aircraft_object.q_to_stab, p_pert)
                q_pert = quat_inv_trans(aircraft_object.q_to_stab, q_pert)
                r_pert = quat_inv_trans(aircraft_object.q_to_stab, r_pert)

            elif frame == "wind":
                p_pert = quat_inv_trans(aircraft_object.q_to_wind, p_pert)
                q_pert = quat_inv_trans(aircraft_object.q_to_wind, q_pert)
                r_pert = quat_inv_trans(aircraft_object.q_to_wind, r_pert)

            # Perturb forward in roll rate
            omega_pert_p_fwd = omega_0+p_pert
            aircraft_object.w = omega_pert_p_fwd
            self.solve_forces(dimensional=False, **kwargs)
            FM_dp_fwd = self._FM

            # Perturb backward in roll rate
            omega_pert_p_bwd = omega_0-p_pert
            aircraft_object.w = omega_pert_p_bwd
            self.solve_forces(dimensional=False, **kwargs)
            FM_dp_bwd = self._FM

            # Perturb forward in pitch rate
            omega_pert_q_fwd = omega_0+q_pert
            aircraft_object.w = omega_pert_q_fwd
            self.solve_forces(dimensional=False, **kwargs)
            FM_dq_fwd = self._FM

            # Perturb backward in pitch rate
            omega_pert_q_bwd = omega_0-q_pert
            aircraft_object.w = omega_pert_q_bwd
            self.solve_forces(dimensional=False, **kwargs)
            FM_dq_bwd = self._FM

            # Perturb forward in yaw rate
            omega_pert_r_fwd = omega_0+r_pert
            aircraft_object.w = omega_pert_r_fwd
            self.solve_forces(dimensional=False, **kwargs)
            FM_dr_fwd = self._FM

            # Perturb backward in yaw rate
            omega_pert_r_bwd = omega_0-r_pert
            aircraft_object.w = omega_pert_r_bwd
            self.solve_forces(dimensional=False, **kwargs)
            FM_dr_bwd = self._FM

            # Reset state
            aircraft_object.w = omega_0
            self._solved = False

            # Compute derivatives
            _, c, b = self.get_aircraft_reference_geometry(aircraft=aircraft_name)
            lat_non_dim = 2*vel_0/b
            lon_non_dim = 2*vel_0/c
            dx_inv = 1/(2*dtheta_dot)

            if body_frame:
                derivs[aircraft_name]["Cx,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cx"]-FM_dp_bwd[aircraft_name]["total"]["Cx"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cy,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cy"]-FM_dp_bwd[aircraft_name]["total"]["Cy"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cz,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cz"]-FM_dp_bwd[aircraft_name]["total"]["Cz"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cl,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cl"]-FM_dp_bwd[aircraft_name]["total"]["Cl"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cm,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cm"]-FM_dp_bwd[aircraft_name]["total"]["Cm"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cn,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cn"]-FM_dp_bwd[aircraft_name]["total"]["Cn"])*dx_inv*lat_non_dim

                derivs[aircraft_name]["Cx,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cx"]-FM_dq_bwd[aircraft_name]["total"]["Cx"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cy,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cy"]-FM_dq_bwd[aircraft_name]["total"]["Cy"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cz,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cz"]-FM_dq_bwd[aircraft_name]["total"]["Cz"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cl,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cl"]-FM_dq_bwd[aircraft_name]["total"]["Cl"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cm,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cm"]-FM_dq_bwd[aircraft_name]["total"]["Cm"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cn,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cn"]-FM_dq_bwd[aircraft_name]["total"]["Cn"])*dx_inv*lon_non_dim

                derivs[aircraft_name]["Cx,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cx"]-FM_dr_bwd[aircraft_name]["total"]["Cx"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cy,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cy"]-FM_dr_bwd[aircraft_name]["total"]["Cy"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cz,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cz"]-FM_dr_bwd[aircraft_name]["total"]["Cz"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cl,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cl"]-FM_dr_bwd[aircraft_name]["total"]["Cl"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cm,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cm"]-FM_dr_bwd[aircraft_name]["total"]["Cm"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cn,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cn"]-FM_dr_bwd[aircraft_name]["total"]["Cn"])*dx_inv*lat_non_dim

            if stab_frame:
                derivs[aircraft_name]["Cx_s,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cx_s"]-FM_dp_bwd[aircraft_name]["total"]["Cx_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cy_s,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cy_s"]-FM_dp_bwd[aircraft_name]["total"]["Cy_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cz_s,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cz_s"]-FM_dp_bwd[aircraft_name]["total"]["Cz_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cl_s,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cl_s"]-FM_dp_bwd[aircraft_name]["total"]["Cl_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cm_s,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cm_s"]-FM_dp_bwd[aircraft_name]["total"]["Cm_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cn_s,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cn_s"]-FM_dp_bwd[aircraft_name]["total"]["Cn_s"])*dx_inv*lat_non_dim

                derivs[aircraft_name]["Cx_s,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cx_s"]-FM_dq_bwd[aircraft_name]["total"]["Cx_s"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cy_s,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cy_s"]-FM_dq_bwd[aircraft_name]["total"]["Cy_s"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cz_s,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cz_s"]-FM_dq_bwd[aircraft_name]["total"]["Cz_s"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cl_s,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cl_s"]-FM_dq_bwd[aircraft_name]["total"]["Cl_s"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cm_s,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cm_s"]-FM_dq_bwd[aircraft_name]["total"]["Cm_s"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cn_s,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cn_s"]-FM_dq_bwd[aircraft_name]["total"]["Cn_s"])*dx_inv*lon_non_dim

                derivs[aircraft_name]["Cx_s,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cx_s"]-FM_dr_bwd[aircraft_name]["total"]["Cx_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cy_s,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cy_s"]-FM_dr_bwd[aircraft_name]["total"]["Cy_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cz_s,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cz_s"]-FM_dr_bwd[aircraft_name]["total"]["Cz_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cl_s,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cl_s"]-FM_dr_bwd[aircraft_name]["total"]["Cl_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cm_s,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cm_s"]-FM_dr_bwd[aircraft_name]["total"]["Cm_s"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cn_s,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cn_s"]-FM_dr_bwd[aircraft_name]["total"]["Cn_s"])*dx_inv*lat_non_dim

            if wind_frame:
                derivs[aircraft_name]["CL,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CL"]-FM_dq_bwd[aircraft_name]["total"]["CL"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["CD,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CD"]-FM_dq_bwd[aircraft_name]["total"]["CD"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["CS,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CS"]-FM_dq_bwd[aircraft_name]["total"]["CS"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cl_w,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cl_w"]-FM_dp_bwd[aircraft_name]["total"]["Cl_w"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cm_w,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cm_w"]-FM_dp_bwd[aircraft_name]["total"]["Cm_w"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cn_w,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cn_w"]-FM_dp_bwd[aircraft_name]["total"]["Cn_w"])*dx_inv*lat_non_dim

                derivs[aircraft_name]["CL,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CL"]-FM_dp_bwd[aircraft_name]["total"]["CL"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["CD,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CD"]-FM_dp_bwd[aircraft_name]["total"]["CD"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["CS,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CS"]-FM_dp_bwd[aircraft_name]["total"]["CS"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cl_w,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cl_w"]-FM_dq_bwd[aircraft_name]["total"]["Cl_w"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cm_w,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cm_w"]-FM_dq_bwd[aircraft_name]["total"]["Cm_w"])*dx_inv*lon_non_dim
                derivs[aircraft_name]["Cn_w,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cn_w"]-FM_dq_bwd[aircraft_name]["total"]["Cn_w"])*dx_inv*lon_non_dim

                derivs[aircraft_name]["CL,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CL"]-FM_dr_bwd[aircraft_name]["total"]["CL"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["CD,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CD"]-FM_dr_bwd[aircraft_name]["total"]["CD"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["CS,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CS"]-FM_dr_bwd[aircraft_name]["total"]["CS"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cl_w,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cl_w"]-FM_dr_bwd[aircraft_name]["total"]["Cl_w"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cm_w,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cm_w"]-FM_dr_bwd[aircraft_name]["total"]["Cm_w"])*dx_inv*lat_non_dim
                derivs[aircraft_name]["Cn_w,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cn_w"]-FM_dr_bwd[aircraft_name]["total"]["Cn_w"])*dx_inv*lat_non_dim

        return derivs


    def control_derivatives(self, aircraft=None, dtheta=0.5, **kwargs):
        """Determines the control derivatives at the current state. Uses 
        a central difference scheme.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the control derivatives 
            of. Defaults to all aircraft in the scene.

        dtheta : float
            The finite difference used to perturb the controls in degrees
            and determine the derivatives. Defaults to 0.5

        body_frame : boolean, optional
            Whether to output results in the body-fixed frame. Defaults to True.

        stab_frame : boolean, optional
            Whether to output results in the stability frame. Defaults to False.

        wind_frame : boolean, optional
            Whether to output results in the wind frame. Defaults to True.

        Returns
        -------
        dict
            A dictionary of control derivatives with respect to deflection in 
            radians.
        """
        derivs = {}

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Determine output frames
        body_frame, stab_frame, wind_frame = self._get_frames(**kwargs)

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}
            aircraft_object = self._airplanes[aircraft_name]
            curr_control_state = copy.deepcopy(aircraft_object.current_control_state)
            pert_control_state = copy.deepcopy(curr_control_state)

            # Loop through available controls
            for control_name in aircraft_object.control_names:

                curr_control_val = curr_control_state.get(control_name, 0.0)

                #Perturb forward
                pert_control_state[control_name] = curr_control_val + dtheta
                aircraft_object.set_control_state(control_state=pert_control_state)
                FM_fwd = self.solve_forces(dimensional=False, **kwargs)

                #Perturb forward
                pert_control_state[control_name] = curr_control_val - dtheta
                aircraft_object.set_control_state(control_state=pert_control_state)
                FM_bwd = self.solve_forces(dimensional=False, **kwargs)

                # Reset state
                pert_control_state[control_name] = curr_control_val
                aircraft_object.set_control_state(control_state=pert_control_state)
                self._solved = False

                # Calculate derivatives
                diff = 2*np.radians(dtheta)

                if body_frame:
                    derivs[aircraft_name]["Cx,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cx"]-FM_bwd[aircraft_name]["total"]["Cx"])/diff
                    derivs[aircraft_name]["Cy,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cy"]-FM_bwd[aircraft_name]["total"]["Cy"])/diff
                    derivs[aircraft_name]["Cz,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cz"]-FM_bwd[aircraft_name]["total"]["Cz"])/diff
                    derivs[aircraft_name]["Cl,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cl"]-FM_bwd[aircraft_name]["total"]["Cl"])/diff
                    derivs[aircraft_name]["Cm,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cm"]-FM_bwd[aircraft_name]["total"]["Cm"])/diff
                    derivs[aircraft_name]["Cn,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cn"]-FM_bwd[aircraft_name]["total"]["Cn"])/diff

                if stab_frame:
                    derivs[aircraft_name]["Cx_s,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cx_s"]-FM_bwd[aircraft_name]["total"]["Cx_s"])/diff
                    derivs[aircraft_name]["Cy_s,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cy_s"]-FM_bwd[aircraft_name]["total"]["Cy_s"])/diff
                    derivs[aircraft_name]["Cz_s,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cz_s"]-FM_bwd[aircraft_name]["total"]["Cz_s"])/diff
                    derivs[aircraft_name]["Cl_s,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cl_s"]-FM_bwd[aircraft_name]["total"]["Cl_s"])/diff
                    derivs[aircraft_name]["Cm_s,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cm_s"]-FM_bwd[aircraft_name]["total"]["Cm_s"])/diff
                    derivs[aircraft_name]["Cn_s,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cn_s"]-FM_bwd[aircraft_name]["total"]["Cn_s"])/diff

                if wind_frame:
                    derivs[aircraft_name]["CL,d"+control_name] = (FM_fwd[aircraft_name]["total"]["CL"]-FM_bwd[aircraft_name]["total"]["CL"])/diff
                    derivs[aircraft_name]["CD,d"+control_name] = (FM_fwd[aircraft_name]["total"]["CD"]-FM_bwd[aircraft_name]["total"]["CD"])/diff
                    derivs[aircraft_name]["CS,d"+control_name] = (FM_fwd[aircraft_name]["total"]["CS"]-FM_bwd[aircraft_name]["total"]["CS"])/diff
                    derivs[aircraft_name]["Cl_w,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cl_w"]-FM_bwd[aircraft_name]["total"]["Cl_w"])/diff
                    derivs[aircraft_name]["Cm_w,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cm_w"]-FM_bwd[aircraft_name]["total"]["Cm_w"])/diff
                    derivs[aircraft_name]["Cn_w,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cn_w"]-FM_bwd[aircraft_name]["total"]["Cn_w"])/diff

        return derivs


    def state_derivatives(self, **kwargs):
        """Determines the derivatives of forces and moments at the current state
        with respect to the 13 element state vector. Uses a central difference scheme.
        These states are:

            Position in Earth-fixed coordinates.
            Velocity in body-fixed coordinates.
            Orientation of the body frame relative to the Earth-fixed frame.
            Angular rate in body-fixed coordinates.

        These derivatives will always be determined using the body-fixed forces and
        moments.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the stability derivatives 
            of. Defaults to all aircraft in the scene.

        dx : float
            The finite difference used to perturb position in either feet or
            meters. Defaults to 0.5.

        dV : float
            The finite difference used to perturb velocity in either ft/s or
            m/s. Defaults to 0.5.

        de : float
            The finite difference used to perturb the orientation quaternion.
            Defaults to 0.001.

        dw : float
            The finite difference used to perturb the angular rates in rad/s.
            Defaults to 0.01.

        Returns
        -------
        dict
            A dictionary of state derivatives.
        """
        derivs= {}

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)
        
        # Get the finite differences
        dx = kwargs.get("dx", 0.5)
        dV = kwargs.get("dV", 0.5)
        de = kwargs.get("de", 0.001)
        dw = kwargs.get("dw", 0.01)

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}

            # Get current state
            v0, w0, p0, q0 = self._airplanes[aircraft_name].get_state()
            orig_state = {
                "position" : p0,
                "velocity" : v0,
                "orientation" : q0,
                "angular_rates" : w0
            }

            # Perturb in velocity
            derivs[aircraft_name].update(self._determine_state_derivs("velocity", "u", 0, dV, orig_state, aircraft_name, **kwargs))
            derivs[aircraft_name].update(self._determine_state_derivs("velocity", "v", 1, dV, orig_state, aircraft_name, **kwargs))
            derivs[aircraft_name].update(self._determine_state_derivs("velocity", "w", 2, dV, orig_state, aircraft_name, **kwargs))

            # Perturb in position
            derivs[aircraft_name].update(self._determine_state_derivs("position", "x_f", 0, dx, orig_state, aircraft_name, **kwargs))
            derivs[aircraft_name].update(self._determine_state_derivs("position", "y_f", 1, dx, orig_state, aircraft_name, **kwargs))
            derivs[aircraft_name].update(self._determine_state_derivs("position", "z_f", 2, dx, orig_state, aircraft_name, **kwargs))

            # Perturb in angular rate
            derivs[aircraft_name].update(self._determine_state_derivs("angular_rates", "p", 0, dw, orig_state, aircraft_name, **kwargs))
            derivs[aircraft_name].update(self._determine_state_derivs("angular_rates", "q", 1, dw, orig_state, aircraft_name, **kwargs))
            derivs[aircraft_name].update(self._determine_state_derivs("angular_rates", "r", 2, dw, orig_state, aircraft_name, **kwargs))

            # Perturb in quaternion
            derivs[aircraft_name].update(self._determine_state_derivs("orientation", "qx", 0, de, orig_state, aircraft_name, **kwargs))
            derivs[aircraft_name].update(self._determine_state_derivs("orientation", "qy", 1, de, orig_state, aircraft_name, **kwargs))
            derivs[aircraft_name].update(self._determine_state_derivs("orientation", "qz", 2, de, orig_state, aircraft_name, **kwargs))
        
            # Reset state
            self._airplanes[aircraft_name].set_state(**orig_state)
            self._solved = False

        return derivs


    def _determine_state_derivs(self, variable, tag, index, perturbation, orig_state, aircraft_name, **kwargs):
        # Perturbs the given index of variable by the perturbation and estimates the derivative

        # Simple perturbations
        pert_state = copy.deepcopy(orig_state)
        if variable in ["position", "velocity", "angular_rates"]:

            # Forward
            pert_state[variable][index] += perturbation
            self._airplanes[aircraft_name].set_state(**pert_state)
            self.solve_forces(nondimensional=False, **kwargs)
            FM_fwd = copy.deepcopy(self._FM)

            # Backward
            pert_state[variable][index] -= 2.0*perturbation
            self._airplanes[aircraft_name].set_state(**pert_state)
            self.solve_forces(nondimensional=False, **kwargs)
            FM_bwd = copy.deepcopy(self._FM)

        # Quaternion perturbation
        else:

            # Get quaternion perturbation
            dq = np.array([1.0, 0.0, 0.0, 0.0])
            dq[index+1] = 0.5*perturbation
            q0 = pert_state["orientation"]

            # Forward
            q_fwd = quat_mult(dq, q0)
            pert_state["orientation"] = q_fwd
            self._airplanes[aircraft_name].set_state(**pert_state)
            self.solve_forces(nondimensional=False, **kwargs)
            FM_fwd = copy.deepcopy(self._FM)

            # Backward
            q_bwd = quat_mult(quat_conj(dq), q0)
            pert_state["orientation"] = q_bwd
            self._airplanes[aircraft_name].set_state(**pert_state)
            self.solve_forces(nondimensional=False, **kwargs)
            FM_bwd = copy.deepcopy(self._FM)

        # Estimate derivative
        derivs = {}
        diff = 0.5/perturbation
        derivs["dFx,d{0}".format(tag)] = (FM_fwd[aircraft_name]["total"]["Fx"]-FM_bwd[aircraft_name]["total"]["Fx"])*diff
        derivs["dFy,d{0}".format(tag)] = (FM_fwd[aircraft_name]["total"]["Fy"]-FM_bwd[aircraft_name]["total"]["Fy"])*diff
        derivs["dFz,d{0}".format(tag)] = (FM_fwd[aircraft_name]["total"]["Fz"]-FM_bwd[aircraft_name]["total"]["Fz"])*diff
        derivs["dMx,d{0}".format(tag)] = (FM_fwd[aircraft_name]["total"]["Mx"]-FM_bwd[aircraft_name]["total"]["Mx"])*diff
        derivs["dMy,d{0}".format(tag)] = (FM_fwd[aircraft_name]["total"]["My"]-FM_bwd[aircraft_name]["total"]["My"])*diff
        derivs["dMz,d{0}".format(tag)] = (FM_fwd[aircraft_name]["total"]["Mz"]-FM_bwd[aircraft_name]["total"]["Mz"])*diff

        return derivs


    def pitch_trim(self, **kwargs):
        """Returns the required angle of attack and elevator deflection for trim at the current state.
        THIS SHOULD ONLY BE USED IN THE CASE OF ONE AIRCRAFT IN THE SCENE AND NO WIND.

        Parameters
        ----------
        pitch_control : str
            The name of the control that should be used to trim in pitch. Defaults to "elevator".

        filename : str
            File to output the results to. Defaults to no file.

        set_trim_state : bool
            If set to True, once trim is determined, the state of the aircraft will be set to this trim state. If
            False, the state of the aircraft will return to what it was before this method was called. Defaults 
            to True.

        verbose : bool
            If set to true, information will be output about the progress of Newton's method. Defaults to 
            False.

        Returns
        -------
        dict
            The angle of attack and deflection of the specified control required to trim the aircraft in 
            pitch in the current state.
        """
        trim_angles = {}

        verbose = kwargs.get("verbose", False)
        if verbose: print("\nTrimming...")

        # Make sure there is only one aircraft in the scene and the wind is constant
        aircraft_names = list(self._airplanes.keys())
        if len(aircraft_names) != 1:
            raise IOError("pitch_trim() may not be used when there is more than one aircraft in the scene.")
        try:
            self._constant_wind
        except:
            raise IOError("pitch_trim() may not be used when the wind is not constant.")

        # Get the aircraft object
        aircraft_name = aircraft_names[0]
        airplane_object = self._airplanes[aircraft_name]

        # Setup output
        pitch_control = kwargs.get("pitch_control", "elevator")
        if verbose:
            print("Trimming {0} using {1}.".format(aircraft_name, pitch_control))
            print("{0:<20}{1:<20}{2:<25}{3:<25}".format("Alpha", pitch_control, "Lift Residual", "Moment Residual"))

        # Store the current orientation, angle of attack, and control deflection
        v_wind = self._get_wind(airplane_object.p_bar)
        alpha_original,_,_ = airplane_object.get_aerodynamic_state(v_wind=v_wind)
        controls_original = copy.copy(airplane_object.current_control_state)

        # Get residuals
        R = self._get_aircraft_pitch_trim_residuals(aircraft_name)

        # Declare initials
        controls = copy.copy(controls_original)
        alpha0 = copy.copy(alpha_original)
        try:
            delta_flap0 = copy.copy(controls_original[pitch_control])
        except KeyError:
            raise IOError("{0} has no {1}. Cannot be trimmed in pitch.".format(aircraft_name, pitch_control))
        J = np.zeros((2,2))

        if verbose: print("{0:<20}{1:<20}{2:<25}{3:<25}".format(alpha0, delta_flap0, R[0], R[1]))

        # Iterate until residuals go to zero.
        while (abs(R)>1e-10).any():

            # Determine Jacobian
            stab_derivs = self.stability_derivatives()
            cont_derivs = self.control_derivatives()
            J[0,0] = stab_derivs[aircraft_name]["CL,a"]
            J[0,1] = cont_derivs[aircraft_name]["CL,d"+pitch_control]
            J[1,0] = stab_derivs[aircraft_name]["Cm,a"]
            J[1,1] = cont_derivs[aircraft_name]["Cm,d"+pitch_control]

            # Calculate update
            delta = np.linalg.solve(J,-R)

            # Update angle of attack
            alpha1 = alpha0 + np.degrees(delta[0])
            airplane_object.set_aerodynamic_state(alpha=alpha1)

            # Update control
            delta_flap1 = delta_flap0 + np.degrees(delta[1])
            controls[pitch_control] = delta_flap1
            airplane_object.set_control_state(controls)

            # Update for next iteration
            alpha0 = alpha1
            delta_flap0 = delta_flap1

            # Determine new residuals
            R = self._get_aircraft_pitch_trim_residuals(aircraft_name=aircraft_name)

            if verbose: print("{0:<20}{1:<20}{2:<25}{3:<25}".format(alpha0, delta_flap0, R[0], R[1]))

        # Store results
        trim_angles[aircraft_name] = {
            "alpha" : alpha1,
            pitch_control : delta_flap1
        }

        # If the user wants, set the state to the new trim state
        set_trim_state = kwargs.get("set_trim_state", True)
        if set_trim_state:
            airplane_object.set_aerodynamic_state(alpha=alpha1)
            self.set_aircraft_control_state({pitch_control : delta_flap1}, aircraft=aircraft_name)

        else: # Return to the original state
            airplane_object.set_aerodynamic_state(alpha=alpha_original)
            self.set_aircraft_control_state(controls_original, aircraft=aircraft_name)

        # Output results to file
        filename = kwargs.get("filename", None)
        if filename is not None:
            with open(filename, 'w') as file_handle:
                json.dump(trim_angles, file_handle, indent=4)

        return trim_angles


    def _get_aircraft_pitch_trim_residuals(self, aircraft_name):
        # Returns the residual force in the earth-fixed z-direction and the residual moment about the body y-axis
        FM = self.solve_forces(dimensional=False)

        # Balance lift and weight with zero moment
        RL = FM[aircraft_name]["total"]["CL"]-self._airplanes[aircraft_name].W/(self._get_aircraft_q_inf(aircraft_name)*self._airplanes[aircraft_name].S_w)
        Rm = FM[aircraft_name]["total"]["Cm"]

        return np.array([RL, Rm])


    def _get_aircraft_q_inf(self, aircraft_name):
        # Returns the dynamic pressure for the given aircraft
        aircraft_object = self._airplanes[aircraft_name]
        rho = self._get_density(aircraft_object.p_bar)
        v_wind = self._get_wind(aircraft_object.p_bar)
        V = np.linalg.norm(aircraft_object.v-v_wind)
        return 0.5*rho*V*V


    def aero_center(self, **kwargs):
        """Returns the location of the aerodynamic center of the aircraft at the current state.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the aerodynamic center 
            of. Defaults to all aircraft in the scene.

        filename : str
            Name of a .json file to output the aerodynamic center locations to.
            Defaults to no file.

        verbose : bool
            If set to true, information will be output about the progress of Newton's method. Defaults to 
            False.

        Returns
        -------
        AC_data : dict
            The location of the aerodynamic center in body-fixed coordinates for each aircraft and the moment coefficient about the AC. Structured as:

            {
                "<AIRCRAFT_NAME>" : {
                    "aero_center" : [x_ac, y_ac, z_ac],
                    "Cm_ac" : Cm_ac
                }
            }
        """

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        ac_loc = {}

        # Loop through aircraft
        verbose = kwargs.get("verbose", False)
        for aircraft_name in aircraft_names:
            if verbose: print("Calculating the aerodynamic center for {0}...".format(aircraft_name))
            airplane_object = self._airplanes[aircraft_name]
            v_wind = self._get_wind(airplane_object.p_bar)

            # Calculate derivatives
            if verbose: print("Calculating derivatives...")
            # Original state
            FM1 = self.solve_forces(dimensional=False)[aircraft_name]["total"]

            a0, B0, V0 = airplane_object.get_aerodynamic_state(v_wind=v_wind)
            delta = 0.5
            delta2 = delta*delta

            # Perturb forward
            airplane_object.set_aerodynamic_state(alpha=a0-delta, beta=B0, velocity=V0, v_wind=v_wind)
            FM0 = self.solve_forces(dimensional=False)[aircraft_name]["total"]

            # Perturb backward
            airplane_object.set_aerodynamic_state(alpha=a0+delta, beta=B0, velocity=V0, v_wind=v_wind)
            FM2 = self.solve_forces(dimensional=False)[aircraft_name]["total"]

            # Reset aircraft state
            airplane_object.set_aerodynamic_state(alpha=a0, beta=B0, velocity=V0, v_wind=v_wind)
            self._solved = False

            # First derivatives
            CA_a = (-FM2["Cx"]+FM0["Cx"])/(2.0*delta)
            CN_a = (-FM2["Cz"]+FM0["Cz"])/(2.0*delta)
            Cm_a = (FM2["Cm"]-FM0["Cm"])/(2.0*delta)

            # Second derivatives
            CA_a2 = (-FM2["Cx"]+2.0*FM1["Cx"]-FM0["Cx"])/delta2
            CN_a2 = (-FM2["Cz"]+2.0*FM1["Cz"]-FM0["Cz"])/delta2
            Cm_a2 = (FM2["Cm"]-2.0*FM1["Cm"]+FM0["Cm"])/delta2

            # Calculate locations (Mech of Flight Eqs. 4.8.29-31)
            if verbose: print("Calculating AC location...")
            denom = CN_a*CA_a2-CA_a*CN_a2
            x_ac = (CA_a*Cm_a2-Cm_a*CA_a2)/denom
            z_ac = (CN_a*Cm_a2-Cm_a*CN_a2)/denom

            # Moment at aerodynamic center
            Cm_ac = FM1["Cm"]-x_ac*FM1["Cz"]+z_ac*FM1["Cx"]

            # Redimensionalize
            l_ref = airplane_object.l_ref_lon
            ac_loc[aircraft_name] = {
                "aero_center" : [-x_ac*l_ref+airplane_object.CG[0], 0.0, -z_ac*l_ref+airplane_object.CG[2]],
                "Cm_ac" : Cm_ac
            }

        # Export
        filename = kwargs.get("filename", None)
        if filename is not None:
            with open(filename, 'w') as output_handle:
                json.dump(ac_loc, output_handle, indent=4)

        return ac_loc


    def distributions(self, **kwargs):
        """Returns various parameters, as well as forces and moments, at each control point for all
        aircraft at the current state. Note that if "correct_sections_for_sweep" (default True) is
        set to True, the section *aerodynamic* properties given here will be the swept section properties.
        
        The following properties are stored as distributions:
        
            "span_frac" : fraction along the span (distance along the LQC projected into the y-z plane)
            "cpx" : control point x location
            "cpy" : control point y location
            "cpz" : control point z location
            "chord" : section geometric chord
            "swept_chord" : section chord normal to the lifting-line (corrected for sweep)
            "twist" : section geometric twist
            "dihedral" : section geometric dihedral
            "sweep" : section geometric sweep
            "aero_sweep" : section aerodynamic sweep (based on the lifting-line)
            "area" : section differential planform area
            "alpha" : angle of attack (corrected for sweep)
            "delta_flap" : flap deflection
            "u" : body-x velocity
            "v" : body-y velocity
            "w" : body-z velocity
            "Re" : Reynolds number
            "M" : Mach number
            "q" : dynamic pressure
            "section_CL" : lift coefficient
            "section_Cm" : moment coefficient
            "section_parasitic_CD" : drag coefficient
            "section_aL0" : zero-lift angle of attack
            "Fx" : body-x force acting on each section
            "Fy" : body-y force acting on each section
            "Fz" : body-z force acting on each section
            "Mx" : body-x moment acting on each section
            "My" : body-y moment acting on each section
            "Mz" : body-z moment acting on each section
            "circ" : circulation


        Parameters
        ----------
        filename : str
            Output file to write the distributions to. Saves as a .txt file. Defaults to no file.

        radians : bool
            Whether to output angular values in radians. Defaults to True. If set to False, all
            angular values will be output in degrees. Note this also affects the plots generated
            by make_plots.

        make_plots : list, optional
            List of keys from the dist dictionary to make plots of. A plot of the parameter as a function 
            of span fraction for each wing segment will then be generated and saved. This can create 
            a lot of plots!

        show_plots : bool, optional
            Whether to show the plots, rather than automatically saving them. Defaults to False.

        Returns
        -------
        dist : dict
            A dictionary containing lists of each parameter at each control point. The distributions are
            organized by aircraft then by wing segment. The nested keys are then each parameter.
        """

        # Make sure the LL equations have been solved in this state
        if not self._solved:
            self.solve_forces(**kwargs)

        # Setup table for saving to .txt file
        index = 0
        filename = kwargs.get("filename", None)
        if filename is not None:
            item_types = [("aircraft", "U18"),
                          ("segment", "U18"),
                          ("span_frac", "float"),
                          ("cpx", "float"),
                          ("cpy", "float"),
                          ("cpz", "float"),
                          ("chord", "float"),
                          ("swept_chord", "float"),
                          ("twist", "float"),
                          ("dihedral", "float"),
                          ("sweep", "float"),
                          ("aero_sweep", "float"),
                          ("area", "float"),
                          ("alpha", "float"),
                          ("delta_flap", "float"),
                          ("u", "float"),
                          ("v", "float"),
                          ("w", "float"),
                          ("Re", "float"),
                          ("M", "float"),
                          ("q", "float"),
                          ("section_CL", "float"),
                          ("section_Cm", "float"),
                          ("section_parasitic_CD", "float"),
                          ("section_aL0","float"),
                          ("Fx", "float"),
                          ("Fy", "float"),
                          ("Fz", "float"),
                          ("Mx", "float"),
                          ("My", "float"),
                          ("Mz", "float"),
                          ("circ", "float")]

            table_data = np.zeros(self._N, dtype=item_types)


        # Loop through airplanes
        radians = kwargs.get("radians", True)
        dist = {}
        for airplane_object in self._airplane_objects:
            airplane_name = airplane_object.name
            dist[airplane_name] = {}

            # Loop through segments
            for segment_object in airplane_object.segments:
                segment_name = segment_object.name
                num_cps = segment_object.N
                cur_slice = slice(index, index+num_cps)
                dist[airplane_name][segment_name] = {}

                # Control point locations
                dist[airplane_name][segment_name]["span_frac"] = list(segment_object.cp_span_locs)
                dist[airplane_name][segment_name]["cpx"] = list(self._PC[cur_slice,0])
                dist[airplane_name][segment_name]["cpy"] = list(self._PC[cur_slice,1])
                dist[airplane_name][segment_name]["cpz"] = list(self._PC[cur_slice,2])

                # Geometry
                dist[airplane_name][segment_name]["chord"] = list(self._c_bar[cur_slice]*self._C_sweep_inv[cur_slice])
                dist[airplane_name][segment_name]["swept_chord"] = list(self._c_bar[cur_slice])
                dist[airplane_name][segment_name]["area"] = list(self._dS[cur_slice])
                if radians:
                    dist[airplane_name][segment_name]["twist"] = list(segment_object.twist_cp)
                    dist[airplane_name][segment_name]["dihedral"] = list(segment_object.dihedral_cp)
                    dist[airplane_name][segment_name]["sweep"] = list(segment_object.sweep_cp)
                    dist[airplane_name][segment_name]["aero_sweep"] = list(self._section_sweep[cur_slice])
                else:
                    dist[airplane_name][segment_name]["twist"] = list(np.degrees(segment_object.twist_cp))
                    dist[airplane_name][segment_name]["dihedral"] = list(np.degrees(segment_object.dihedral_cp))
                    dist[airplane_name][segment_name]["sweep"] = list(np.degrees(segment_object.sweep_cp))
                    dist[airplane_name][segment_name]["aero_sweep"] = list(np.degrees(self._section_sweep[cur_slice]))

                # Airfoil info
                if radians:
                    if self._use_swept_sections:
                        dist[airplane_name][segment_name]["section_aL0"] = list(self._aL0[cur_slice]*self._C_sweep_inv[cur_slice])
                    else:
                        dist[airplane_name][segment_name]["section_aL0"] = list(self._aL0[cur_slice])
                    dist[airplane_name][segment_name]["alpha"] = list(self._alpha[cur_slice])
                    dist[airplane_name][segment_name]["delta_flap"] = list(segment_object._delta_flap)

                else:
                    if self._use_swept_sections:
                        dist[airplane_name][segment_name]["section_aL0"] = list(np.degrees(self._aL0[cur_slice]*self._C_sweep_inv[cur_slice]))
                    else:
                        dist[airplane_name][segment_name]["section_aL0"] = list(np.degrees(self._aL0[cur_slice]))
                    dist[airplane_name][segment_name]["alpha"] = list(np.degrees(self._alpha[cur_slice]))
                    dist[airplane_name][segment_name]["delta_flap"] = list(np.degrees(segment_object._delta_flap))

                # Section coefficients
                dist[airplane_name][segment_name]["section_CL"] = list(self._CL[cur_slice])
                dist[airplane_name][segment_name]["section_Cm"] = list(self._Cm[cur_slice])
                dist[airplane_name][segment_name]["section_parasitic_CD"] = list(self._CD[cur_slice])

                # Section force and moment components
                dist[airplane_name][segment_name]["Fx"] = list(self._dF_inv[cur_slice,0]+self._dF_visc[cur_slice,0])
                dist[airplane_name][segment_name]["Fy"] = list(self._dF_inv[cur_slice,1]+self._dF_visc[cur_slice,1])
                dist[airplane_name][segment_name]["Fz"] = list(self._dF_inv[cur_slice,2]+self._dF_visc[cur_slice,2])
                dist[airplane_name][segment_name]["Mx"] = list(self._dM_inv[cur_slice,0]+self._dM_visc[cur_slice,0])
                dist[airplane_name][segment_name]["My"] = list(self._dM_inv[cur_slice,1]+self._dM_visc[cur_slice,1])
                dist[airplane_name][segment_name]["Mz"] = list(self._dM_inv[cur_slice,2]+self._dM_visc[cur_slice,2])
                dist[airplane_name][segment_name]["circ"] = list(self._gamma[cur_slice])

                # Atmospheric properties
                v = quat_trans(airplane_object.q, self._v_i[cur_slice,:])
                dist[airplane_name][segment_name]["u"] = list(v[:,0])
                dist[airplane_name][segment_name]["v"] = list(v[:,1])
                dist[airplane_name][segment_name]["w"] = list(v[:,2])
                dist[airplane_name][segment_name]["Re"] = list(self._Re[cur_slice])
                dist[airplane_name][segment_name]["M"] = list(self._M[cur_slice])
                if self._use_in_plane:
                    dist[airplane_name][segment_name]["q"] = list(self._redim_in_plane[cur_slice]/self._dS[cur_slice])
                else:
                    dist[airplane_name][segment_name]["q"] = list(self._redim_full[cur_slice]/self._dS[cur_slice])

                # Save to data table
                if filename is not None:
                    # Names
                    table_data[cur_slice]["aircraft"] = airplane_name
                    table_data[cur_slice]["segment"] = segment_name

                    # Control point locations
                    table_data[cur_slice]["span_frac"] = dist[airplane_name][segment_name]["span_frac"]
                    table_data[cur_slice]["cpx"] = dist[airplane_name][segment_name]["cpx"]
                    table_data[cur_slice]["cpy"] = dist[airplane_name][segment_name]["cpy"]
                    table_data[cur_slice]["cpz"] = dist[airplane_name][segment_name]["cpz"]

                    # Geometry
                    table_data[cur_slice]["chord"] = dist[airplane_name][segment_name]["chord"]
                    table_data[cur_slice]["swept_chord"] = dist[airplane_name][segment_name]["swept_chord"]
                    table_data[cur_slice]["twist"] = dist[airplane_name][segment_name]["twist"]
                    table_data[cur_slice]["dihedral"] = dist[airplane_name][segment_name]["dihedral"]
                    table_data[cur_slice]["sweep"] = dist[airplane_name][segment_name]["sweep"]
                    table_data[cur_slice]["aero_sweep"] = dist[airplane_name][segment_name]["aero_sweep"]
                    table_data[cur_slice]["area"] = dist[airplane_name][segment_name]["area"]

                    # Airfoil info
                    table_data[cur_slice]["alpha"] = dist[airplane_name][segment_name]["alpha"]
                    table_data[cur_slice]["delta_flap"] = dist[airplane_name][segment_name]["delta_flap"]
                    table_data[cur_slice]["Re"] = dist[airplane_name][segment_name]["Re"]
                    table_data[cur_slice]["M"] = dist[airplane_name][segment_name]["M"]
                    table_data[cur_slice]["q"] = dist[airplane_name][segment_name]["q"]
                    table_data[cur_slice]["u"] = dist[airplane_name][segment_name]["u"]
                    table_data[cur_slice]["v"] = dist[airplane_name][segment_name]["v"]
                    table_data[cur_slice]["w"] = dist[airplane_name][segment_name]["w"]

                    # Section coefficients
                    table_data[cur_slice]["section_CL"] = dist[airplane_name][segment_name]["section_CL"]
                    table_data[cur_slice]["section_Cm"] = dist[airplane_name][segment_name]["section_Cm"]
                    table_data[cur_slice]["section_parasitic_CD"] = dist[airplane_name][segment_name]["section_parasitic_CD"]
                    table_data[cur_slice]["section_aL0"] = dist[airplane_name][segment_name]["section_aL0"]

                    # Section force and moment components
                    table_data[cur_slice]["Fx"] = dist[airplane_name][segment_name]["Fx"]
                    table_data[cur_slice]["Fy"] = dist[airplane_name][segment_name]["Fy"]
                    table_data[cur_slice]["Fz"] = dist[airplane_name][segment_name]["Fz"]
                    table_data[cur_slice]["Mx"] = dist[airplane_name][segment_name]["Mx"]
                    table_data[cur_slice]["My"] = dist[airplane_name][segment_name]["My"]
                    table_data[cur_slice]["Mz"] = dist[airplane_name][segment_name]["Mz"]
                    table_data[cur_slice]["circ"] = dist[airplane_name][segment_name]["circ"]

                index += num_cps

        # Save distributions to .txt file
        if filename is not None:
            
            # Define header and output format
            header = "{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}".format(
                "Aircraft", "Segment", "Span Fraction", "Control (x)", "Control (y)", "Control (z)", "Chord", "Swept Chord", "Twist", "Dihedral", "Sweep", "Aero Sweep", "Area", "Alpha",
                "Flap Defl.", "u", "v", "w", "Re", "M", "q", "CL", "Cm", "Parasitic CD", "Zero-Lift Alpha", "Fx", "Fy", "Fz", "Mx", "My", "Mz", "Circ")
            format_string = "%-20s %-20s %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e"

            # Save
            np.savetxt(filename, table_data, fmt=format_string, header=header)

        # Create plots specified by the user
        make_plots = kwargs.get("make_plots", [])
        for param in make_plots:
            for aircraft_object in self._airplane_objects:
                for segment_name, segment_dist in dist[aircraft_object.name].items():
                    plt.figure()
                    plt.plot(segment_dist["span_frac"], segment_dist[param])
                    plt.xlabel("Span Fraction")
                    plt.ylabel(param)
                    plt.title(segment_name)
                    if kwargs.get("show_plots", False):
                        plt.show()
                    else:
                        plt.savefig("{0}_{1}_{2}_vs_span_fraction".format(aircraft_object.name, segment_name, param))
                    plt.close()

        return dist


    def get_aircraft_reference_geometry(self, aircraft=None):
        """Returns the reference geometries for the specified aircraft.

        Parameters
        ----------
        aircraft : str
            The name of the aircraft to get the reference params for. Does
            not need to be specified if there is only one aircraft in the 
            scene. Only one may be specified.

        Returns
        -------
        S_w : float
            Reference area
        
        l_ref_lon : float
            Longitudinal reference length

        l_ref_lat : float
            Lateral reference length
        """

        # Specify the only aircraft if not already specified
        if aircraft is None:
            if self._num_aircraft == 1:
                aircraft = list(self._airplanes.keys())[0]
            else:
                raise IOError("Aircraft name must be specified if there is more than one aircraft in the scene.")

        airplane_object = self._airplanes[aircraft]
        return airplane_object.S_w, airplane_object.l_ref_lon, airplane_object.l_ref_lat


    def export_stl(self, **kwargs):
        """Generates a 3D model of the aircraft. If only one aircraft is specified, the model is centered on that
        aircraft's origin. If more than one aircraft is specified, the model is centered on the origin of the earth-
        fixed coordinate system.

        Parameters
        ----------
        filename: str
            Name of the file to export the model to. Must be .stl.

        section_resolution : int, optional
            Number of points to use in dicretizing the airfoil section outlines. Defaults to 200. Note this is the
            number of outline points where two exist at the trailing edge. Thus the number of panels will be one less
            than this number.

        aircraft : str or list, optional
            Name(s) of the aircraft to include in the model. Defaults to all aircraft in the scene.

        close_te : bool, optional
            Whether to force the trailing edge to be sealed. Defaults to true
        """

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Model of single aircraft
        if len(aircraft_names) == 1:
            self._airplanes[aircraft_names[0]].export_stl(**kwargs)
            return

        # Check for .stl file
        filename = kwargs.get("filename")
        if ".stl" not in filename:
            raise IOError("{0} is not a .stl file.".format(filename))

        # Multiple aircraft
        else:
            num_facets = 0
            vector_dict = {}

            # Loop through aircraft
            for aircraft_name in aircraft_names:
                airplane_object = self._airplanes[aircraft_name]
                vector_dict[aircraft_name] = {}

                # Loop through segments
                for segment_name, segment_object in airplane_object.wing_segments.items():
                    vectors = segment_object.get_stl_vectors(**kwargs)
                    vector_dict[aircraft_name][segment_name] = airplane_object.p_bar+quat_inv_trans(airplane_object.q, vectors)
                    num_facets += int(vectors.shape[0]/3)

            # Allocate mesh
            model_mesh = mesh.Mesh(np.zeros(num_facets, dtype=mesh.Mesh.dtype))

            # Store vectors
            index = 0
            for aircraft_name in aircraft_names:
                airplane_object = self._airplanes[aircraft_name]
                for segment_name, segment_object in airplane_object.wing_segments.items():
                    num_segment_facets = int(vector_dict[aircraft_name][segment_name].shape[0]/3)
                    for i in range(index, index+num_segment_facets):
                        for j in range(3):
                            model_mesh.vectors[i][j] = vector_dict[aircraft_name][segment_name][3*(i-index)+j]
                    index += num_segment_facets

            # Export
            model_mesh.save(filename)


    def MAC(self, **kwargs):
        """Returns the mean aerodynamic chord (MAC) for the specified aircraft.

        Parameters
        ----------
        aircraft : str
            The name of the aircraft to get the reference params for. Does
            not need to be specified if there is only one aircraft in the 
            scene.

        filename : str
            JSON file to export the MAC data to. Defaults to None.

        Returns
        -------
        MAC : dict
            MAC data for each aircraft. Structured as 

                {
                    "<AIRCRAFT_NAME>" : {
                        "length" : mean aerodynamic chord length,
                        "C_point" : location of the quarter chord of the MAC determined by Eq. 2.6.2 from Nickel and Wohlfahrt "Tailless Aircraft"
                    }
                }
        """

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        MAC = {}

        # Loop through aircraft
        for aircraft_name in aircraft_names:
            MAC[aircraft_name] = self._airplanes[aircraft_name].get_MAC()

        # Export
        filename = kwargs.get("filename", None)
        if filename is not None:
            with open(filename, 'w') as dump_handle:
                json.dump(MAC, dump_handle)

        return MAC


    def export_stp(self, **kwargs):
        """Creates a .stp file representing each lifting surface of the specified aircraft.
        NOTE: FreeCAD must be installed and configured to use this function.

        Parameters
        ----------
        aircraft : str, optional
            The aircraft to export a .stp file of. Defaults to all aircraft in the scene.

        file_tag : str, optional
            Optional tag to prepend to output filename default. The output files will be named "<AIRCRAFT_NAME>_<WING_NAME>.stp".

        section_resolution : int, optional
            Number of points to use in discretizing the airfoil section outline. Defaults to 200.
        
        spline : bool, optional
            Whether the wing segment sections should be represented using splines. This can cause issues with some geometries/CAD 
            packages. Defaults to False.

        maintain_sections : bool, optional
            Whether the wing segment sections should be preserved in the loft. Defaults to True.

        close_te : bool, optional
            Whether to force the trailing edge to be sealed. Defaults to true
        """

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Loop through aircraft
        for aircraft_name in aircraft_names:
            self._airplanes[aircraft_name].export_stp(**kwargs)


    def export_dxf(self, **kwargs):
        """Creates a .dxf file representing each lifting surface of the specified aircraft.

        Parameters
        ----------
        aircraft : str
            The aircraft to export .dxf files of.

        file_tag : str, optional
            Optional tag to prepend to output filename default. The output files will be named "<AIRCRAFT_NAME>_<WING_NAME>.dxf".

        section_resolution : int, optional
            Number of points to use in discretizing the airfoil section outline. Defaults to 200.
        
        number_guide_curves : int
            Number of guidecurves to create. Defaults to 2 (one at the leading edge, one at the trailing edge).
        
        dxf_line_type : str
            Type of line to be used in the .dxf file creation. Options include 'line', 'spline', and 'polyline'. Defaults to 'spline'.
        """
        
        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Loop through aircraft
        for aircraft_name in aircraft_names:
            self._airplanes[aircraft_name].export_dxf(**kwargs)


    def export_pylot_model(self, **kwargs):
        """Creates a JSON object containing a linearized model of the aircraft to use as input
        for Pylot (www.github.com/usuaero/Pylot). Any information not available to MachupX but
        required for Pylot will be filled with "PLEASE SPECIFY" and must be changed by the 
        user before the input can be used for Pylot. Note, this can only be used if there is 
        one aircraft in the scene.

        We designed the input files for Pylot to be cross-compatible with MachUpX. With this in
        mind, if values are already specified in the input but those values are not used in MachUpX,
        they will still be included in the input file exported here.

        Note, this will set the aircraft state to zero aerodynamic angles and zero control deflections.

        Parameters
        ----------
        filename : str, optional
            Name of the JSON file to write the model to. Must be ".json". Defaults to 
            "<AIRCRAFT_NAME>_linearized.json".

        inertia : dict, optional
            Moments of inertia for the aircraft, formatted as

                {
                    "Ixx" : <VALUE>,
                    "Iyy" : <VALUE>,
                    "Izz" : <VALUE>,
                    "Ixy" : <VALUE>,
                    "Ixz" : <VALUE>,
                    "Iyz" : <VALUE>
                }
            
            If not specified, this will be left blank for the user to specify after the fact.
            Alternatively, if "inertia" was already part of the aircraft input, it will remain
            the same as inputted.

        angular_momentum : list, optional
            Angular momentum vector. Defaults to [0.0, 0.0, 0.0]. Alternatively, if "angular_momentum"
            was already part of the aircraft input, it will remain the same as inputted.

        stall_angle_of_attack : float, optional
            Angle of attack in degrees at which the aircraft stalls.
        
        stall_sideslip_angle : float, optional
            Sideslip angle in degrees at which the aircraft stalls laterally.

        controller_type : str, optional
            The controller that will be used with the exported model. Can be "keyboard", "joystick",
            "user_defined", or "time_sequence". This affects whether certain inputs unknown to MachUpX
            are marked "PLEASE SPECIFY". If not given, all such keys will be marked "PLEASE SPECIFY".

        velocity : float, optional
            Velocity at which to evaluate the model. Should not have any effect unless Mach and Reynolds
            number effects are included. Defaults to 100.

        set_accel_derivs : bool, optional
            Whether to set derivatives with respect to vertical and lateral acceleration to zero. Defaults
            to False, in which case the user must specify these.
        """

        # Make sure there is only one aircraft in the scene
        aircraft_names = list(self._airplanes.keys())
        if len(aircraft_names) != 1:
            raise IOError("export_pylot_model() may not be used when there is more than one aircraft in the scene.")

        # Initialize
        aircraft_name = aircraft_names[0]
        aircraft_object = self._airplanes[aircraft_name]
        model_dict = copy.deepcopy(aircraft_object._input_dict)
        model_dict.pop("wings")
        model_dict.pop("airfoils")

        # Store params
        model_dict["units"] = self._unit_sys
        model_dict["CG"] = list(aircraft_object.CG)
        model_dict["weight"] = float(aircraft_object.W)
        model_dict["reference"] = {
            "area" : float(aircraft_object.S_w),
            "longitudinal_length" : float(aircraft_object.l_ref_lon),
            "lateral_length" : float(aircraft_object.l_ref_lat)
        }

        # Store inertia and angular momentum
        try:
            model_dict["inertia"]
        except KeyError:
            def_inertia = {
                "Ixx" : "PLEASE SPECIFY",
                "Iyy" : "PLEASE SPECIFY",
                "Izz" : "PLEASE SPECIFY",
                "Ixy" : "PLEASE SPECIFY",
                "Ixz" : "PLEASE SPECIFY",
                "Iyz" : "PLEASE SPECIFY"
            }
            model_dict["inertia"] = kwargs.get("inertia", def_inertia)
        try:
            model_dict["angular_momentum"]
        except KeyError:
            model_dict["angular_momentum"] = list(kwargs.get("angular_momentum", [0.0, 0.0, 0.0]))

        # Inform the user which control parameters need to be specified
        control_type = kwargs.get("controller_type", None)
        try:
            for key, value in model_dict["controls"].items():
                if control_type == "keyboard" or control_type == "joystick" or control_type == None:
                    try:
                        value["max_deflection"] = value["max_deflection"]
                    except KeyError:
                        if control_type == None:
                            value["max_deflection"] = "PLEASE SPECIFY"
                        pass
                    value["input_axis"] = value.get("input_axis", "PLEASE SPECIFY")
                
                if control_type == "time_sequence" or control_type == None:
                    value["column_index"] = value.get("column_index", "PLEASE SPECIFY")

        except KeyError:
            pass

        # Specify model type
        model_dict["aero_model"] = {
            "type" : "linearized_coefficients"
        }
        try:
            model_dict["aero_model"]["stall_angle_of_attack"] = kwargs["stall_angle_of_attack"]
        except KeyError:
            pass
        try:
            model_dict["aero_model"]["stall_sideslip_angle"] = kwargs["stall_sideslip_angle"]
        except KeyError:
            pass

        # Set reference state at zero sideslip and angle of attack, zero control deflections, and zero angular rates
        V_ref = kwargs.get("velocity", 100)
        self.set_aircraft_state(state={"velocity" : V_ref})
        self.set_aircraft_control_state()

        # Get forces and derivatives at reference state
        FM_ref = self.solve_forces(dimensional=False)
        derivs_ref = self.derivatives()

        # Get reference coefficients, stability and damping derivatives
        model_dict["coefficients"] = {}
        model_dict["coefficients"]["CL0"] = float(FM_ref[aircraft_name]["total"]["CL"])
        model_dict["coefficients"]["Cm0"] = float(FM_ref[aircraft_name]["total"]["Cm"])
        model_dict["coefficients"]["CL,a"] = float(derivs_ref[aircraft_name]["stability"]["CL,a"])
        model_dict["coefficients"]["Cm,a"] = float(derivs_ref[aircraft_name]["stability"]["Cm,a"])
        model_dict["coefficients"]["CS,b"] = float(derivs_ref[aircraft_name]["stability"]["CS,b"])
        model_dict["coefficients"]["Cl,b"] = float(derivs_ref[aircraft_name]["stability"]["Cl,b"])
        model_dict["coefficients"]["Cn,b"] = float(derivs_ref[aircraft_name]["stability"]["Cn,b"])
        model_dict["coefficients"]["CS,p_bar"] = float(derivs_ref[aircraft_name]["damping"]["CS,pbar"])
        model_dict["coefficients"]["Cl,p_bar"] = float(derivs_ref[aircraft_name]["damping"]["Cl,pbar"])
        model_dict["coefficients"]["Cn,p_bar"] = float(derivs_ref[aircraft_name]["damping"]["Cn,pbar"])
        model_dict["coefficients"]["CL,q_bar"] = float(derivs_ref[aircraft_name]["damping"]["CL,qbar"])
        model_dict["coefficients"]["CD,q_bar"] = float(derivs_ref[aircraft_name]["damping"]["CD,qbar"])
        model_dict["coefficients"]["Cm,q_bar"] = float(derivs_ref[aircraft_name]["damping"]["Cm,qbar"])
        model_dict["coefficients"]["CS,r_bar"] = float(derivs_ref[aircraft_name]["damping"]["CS,rbar"])
        model_dict["coefficients"]["Cl,r_bar"] = float(derivs_ref[aircraft_name]["damping"]["Cl,rbar"])
        model_dict["coefficients"]["Cn,r_bar"] = float(derivs_ref[aircraft_name]["damping"]["Cn,rbar"])

        # Specify coefficients MachUpX doesn't know about
        if kwargs.get("set_accel_derivs", False):
            val = 0.0
        else:
            val = "PLEASE SPECIFY"
        model_dict["coefficients"]["CL,a_hat"] = val
        model_dict["coefficients"]["CD,a_hat"] = val
        model_dict["coefficients"]["Cm,a_hat"] = val
        model_dict["coefficients"]["CS,b_hat"] = val
        model_dict["coefficients"]["Cl,b_hat"] = val
        model_dict["coefficients"]["Cn,b_hat"] = val

        # Specify control derivatives
        for control_name in aircraft_object.control_names:
            model_dict["coefficients"][control_name] = {}
            model_dict["coefficients"][control_name]["CL"] = float(derivs_ref[aircraft_name]["control"]["CL,d"+control_name])
            model_dict["coefficients"][control_name]["CD"] = float(derivs_ref[aircraft_name]["control"]["CD,d"+control_name])
            model_dict["coefficients"][control_name]["CS"] = float(derivs_ref[aircraft_name]["control"]["CS,d"+control_name])
            model_dict["coefficients"][control_name]["Cl"] = float(derivs_ref[aircraft_name]["control"]["Cl,d"+control_name])
            model_dict["coefficients"][control_name]["Cm"] = float(derivs_ref[aircraft_name]["control"]["Cm,d"+control_name])
            model_dict["coefficients"][control_name]["Cn"] = float(derivs_ref[aircraft_name]["control"]["Cn,d"+control_name])

        # Evaluate drag polar in alpha
        num_points = 21
        alphas = np.linspace(-10, 10, num_points)
        CL = np.zeros(num_points)
        CD = np.zeros(num_points)
        for i, alpha in enumerate(alphas):
            self.set_aircraft_state(state={"velocity" : V_ref, "alpha" : alpha})
            FM = self.solve_forces(dimensional=False, body_frame=False)
            CL[i] = FM[aircraft_name]["total"]["CL"]
            CD[i] = FM[aircraft_name]["total"]["CD"]

        coefs = np.polyfit(CL, CD, 2)
        model_dict["coefficients"]["CD0"] = float(coefs[2])
        model_dict["coefficients"]["CD1"] = float(coefs[1])
        model_dict["coefficients"]["CD2"] = float(coefs[0])

        # Determine zero-lift aoa
        coefs = np.polyfit(alphas, CL, 1)
        a_L0 = -coefs[1]/coefs[0]

        # Evaluate drag polar in beta at zero-lift angle of attack
        num_points = 21
        betas = np.linspace(-10, 10, num_points)
        CS = np.zeros(num_points)
        CD = np.zeros(num_points)
        for i, beta in enumerate(betas):
            self.set_aircraft_state(state={"velocity" : V_ref, "alpha" : a_L0, "beta" : beta})
            FM = self.solve_forces(dimensional=False, body_frame=False)
            CS[i] = FM[aircraft_name]["total"]["CS"]
            CD[i] = FM[aircraft_name]["total"]["CD"]

        coefs = np.polyfit(CS, CD, 2)
        model_dict["coefficients"]["CD3"] = float(coefs[0])

        # Put in placeholder engine
        placeholder = {
            "placeholder_engine" : {
            }
        }
        model_dict["engines"] = model_dict.get("engines", placeholder)

        # Put in placeholder landing gear
        placeholder = {
            "placeholder_landing_gear" : {
            }
        }
        model_dict["landing_gear"] = model_dict.get("landing_gear", placeholder)

        # Export model
        filename = kwargs.get("filename", aircraft_name+"_linearized.json")
        with open(filename, 'w') as output_handle:
            json.dump(model_dict, output_handle, indent=4)


    def _get_aircraft(self, **kwargs):
        # Generates a list of aircraft to perform the function on

        aircraft = kwargs.get("aircraft", None)

        # All aircraft
        if aircraft is None:
            aircraft_names = list(self._airplanes.keys())

        # Some aircraft
        elif isinstance(aircraft, list):
            aircraft_names = copy.copy(aircraft)

        # One aircraft
        elif isinstance(aircraft, str):
            aircraft_names = [aircraft]

        else:
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft))

        return aircraft_names


    def out_gamma(self):
        """Plots the induced velocities and writes the circulation distribution to a file.

        Author: Francois Fortin
        """

        # Get span locations
        y_locs = self._PC[:,1]

        with open('gamma_dist.txt','w') as output_handle:

            # Output gammas
            for i in range(self._N):
                print(i, y_locs[i], self._gamma[i], file=output_handle)
                
            # Check V_i is computed
            if not hasattr(self, "_V_i"):
                self._calc_v_i()
                self._V_i = np.linalg.norm(self._v_i, axis=-1)

            # Output velocities
            print('i  y  v_i  V_i', file=output_handle)
            for i in range(self._N):
                print(y_locs[i], self._v_i[i,:], self._V_i[i], file=output_handle)          

            # Plot velocity magnitudes
            plt.figure()
            plt.plot(y_locs, self._V_i)
            plt.ylabel('V_i')
            plt.show()

            # Plot gamma
            plt.figure()
            plt.plot(y_locs, self._gamma)
            plt.ylabel('gamma')
            plt.show()


    def set_err_state(self, **kwargs):
        """Sets how errors are to be handled.

        Each error type can be set to "raise", "warn", or "ignore". If set to "raise", the 
        error will be raised and execution will be interrupted. If set to "warn", a warning
        will be given, but execution will be allowed to continue. If set to "ignore", no 
        message will be given and execution will continue. This can only be set for custom
        exceptions defined for MachUpX and AirfoilDatabase.

        All will default to "raise" if not specified.

        Parameters
        ----------
        not_converged : str, optional
            How to handle the SolverNotConvergedError.

        database_bounds : str, optional
            How to handle the DatabaseBoundsError.
        """

        # Set error state
        self._err_state = {}
        self._err_state["not_converged"] = kwargs.get("not_converged", "raise")
        self._err_state["database_bounds"] = kwargs.get("database_bounds", "raise")


    def _handle_error(self, error):
        # Handles an error according to the error state

        # Has to be a custom exception
        if isinstance(error, SolverNotConvergedError):
            key = "not_converged"
        elif isinstance(error, DatabaseBoundsError):
            key = "database_bounds"
        else:
            raise error

        # Handle
        instruction = self._err_state[key]
        if instruction == "raise":
            raise error
        elif instruction == "warn":
            warnings.warn(str(error))
        elif instruction == "ignore":
            return
        else:
            raise RuntimeError("MachUpX Scene got an incorrect error handling instruction. '{0}' is invalid.".format(instruction))
        

    def target_CL(self, **kwargs):
        """Determines the angle of attack necessary to produce the specified lift coefficient
        with the specified control deflections. MAY ONLY BE USED IF THERE IS ONE AIRCRAFT IN 
        THE SCENE AND THE WIND IS CONSTANT.

        Parameters
        ----------
        CL : float
            Target lift coefficient.

        control_state : dict, optional
            Control deflections. Defaults to no deflections.

        set_state : bool, optional
            Whether to set the state of the aircraft to the angle of attack determined.

        filename : str, optional
            File to output results to. Defaults to no file.

        verbose : bool, optional
            Whether to output the progress of the iterative solver. Defaults to False.

        Returns
        -------
        alpha : float
            Angle of attack at the given CL.
        """

        # Make sure there is only one aircraft in the scene and the wind is constant
        aircraft_names = list(self._airplanes.keys())
        if len(aircraft_names) != 1:
            raise IOError("target_CL() may not be used when there is more than one aircraft in the scene.")
        try:
            self._constant_wind
        except:
            raise IOError("target_CL() may not be used when the wind is not constant.")

        # Get parameters
        alpha = 0.0
        verbose = kwargs.get("verbose", False)
        CL_target = kwargs.get("CL")
        controls = kwargs.get("control_state", {})
        if verbose: print("\nSetting angle of attack for CL={0}...".format(CL_target))

        # Get the aircraft object
        aircraft_name = aircraft_names[0]
        airplane_object = self._airplanes[aircraft_name]

        # Setup output
        if verbose:
            print("{0:<25}{1:<25}".format("Alpha", "CL"))

        # Store the current orientation, angle of attack, and control deflection
        v_wind = self._get_wind(airplane_object.p_bar)
        alpha_original,_,_ = airplane_object.get_aerodynamic_state(v_wind=v_wind)
        controls_original = copy.copy(airplane_object.current_control_state)

        # Get residuals
        airplane_object.set_aerodynamic_state(alpha=alpha)
        airplane_object.set_control_state(controls)
        CL = self.solve_forces(dimensional=False)[aircraft_name]["total"]["CL"]

        if verbose: print("{0:<25}{1:<25}".format(alpha, CL))

        # Iterate until residuals go to zero.
        while (abs(CL-CL_target)>1e-10).any():

            # Perturb forward in alpha
            airplane_object.set_aerodynamic_state(alpha=alpha+0.005)
            CL_fwd = self.solve_forces(dimensional=False)[aircraft_name]["total"]["CL"]

            # Perturb backward in alpha
            airplane_object.set_aerodynamic_state(alpha=alpha-0.005)
            CL_bwd = self.solve_forces(dimensional=False)[aircraft_name]["total"]["CL"]

            # Determine update
            CLa = (CL_fwd-CL_bwd)/0.01
            alpha += (CL_target-CL)/CLa

            # Determine new residuals
            airplane_object.set_aerodynamic_state(alpha=alpha)
            CL = self.solve_forces(dimensional=False)[aircraft_name]["total"]["CL"]

            if verbose: print("{0:<25}{1:<25}".format(alpha, CL))

        # If the user wants, set the state to the new trim state
        set_state = kwargs.get("set_state", True)
        if set_state:
            airplane_object.set_aerodynamic_state(alpha=alpha)
            self.set_aircraft_control_state(control_state=controls, aircraft=aircraft_name)

        else: # Return to the original state
            airplane_object.set_aerodynamic_state(alpha=alpha_original)
            self.set_aircraft_control_state(controls_original, aircraft=aircraft_name)

        # Output results to file
        filename = kwargs.get("filename", None)
        if filename is not None:
            with open(filename, 'w') as file_handle:
                json.dump({"CL" : CL_target, "alpha" : alpha}, file_handle, indent=4)

        return alpha
