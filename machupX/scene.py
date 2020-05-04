from .helpers import quat_inv_trans, quat_trans, check_filepath, import_value
from .airplane import Airplane
from .standard_atmosphere import StandardAtmosphere

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
        self._airplane_names = []
        self._N = 0
        self._num_aircraft = 0

        # Track whether the scene in its current state has been solved
        # Should be set to False any time any state variable is changed without immediately thereafter calling solve_forces()
        self._solved = False

        # Import information from the input
        self._load_params(scene_input)


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
        self._solver_type = solver_params.get("type", "linear")
        self._solver_convergence = solver_params.get("convergence", 1e-10)
        self._solver_relaxation = solver_params.get("relaxation", 1.0)
        self._max_solver_iterations = solver_params.get("max_iterations", 100)

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
                return self._std_atmos.rho(-position[2])
            
        # Array
        elif isinstance(rho, np.ndarray):
            self._density_data = rho

            # Create getters
            if self._density_data.shape[1] is 2: # Density profile

                def density_getter(position):
                    return np.interp(-position[2], self._density_data[:,0], self._density_data[:,1])

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
                    self._wind_field_x_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3],self._wind_data[:,3])
                    self._wind_field_y_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3],self._wind_data[:,4])
                    self._wind_field_z_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3],self._wind_data[:,5])

                    def wind_getter(position):
                        Vx = self._wind_field_x_interpolator(position).item()
                        Vy = self._wind_field_y_interpolator(position).item()
                        Vz = self._wind_field_z_interpolator(position).item()
                        return [Vx, Vy, Vz]

                elif self._wind_data.shape[1] is 4: # wind profile

                    def wind_getter(position):
                        Vx =  np.interp(-position[2], self._wind_data[:,0], self._wind_data[:,1]).item()
                        Vy =  np.interp(-position[2], self._wind_data[:,0], self._wind_data[:,2]).item()
                        Vz =  np.interp(-position[2], self._wind_data[:,0], self._wind_data[:,3]).item()
                        return [Vx, Vy, Vz]

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
        self._max_thickness = np.zeros(self._N)
        self._max_camber = np.zeros(self._N)

        # Node locations
        self._P0 = np.zeros((self._N,self._N,3)) # Inbound vortex node location; takes into account effective LAC where appropriate
        self._P0_joint = np.zeros((self._N,self._N,3)) # Inbound vortex joint node location
        self._P1 = np.zeros((self._N,self._N,3)) # Outbound vortex node location
        self._P1_joint = np.zeros((self._N,self._N,3)) # Outbound vortex joint node location

        # Spatial node vectors
        self._r_0 = np.empty((self._N,self._N,3))
        self._r_1 = np.empty((self._N,self._N,3))
        self._r_0_joint = np.empty((self._N,self._N,3))
        self._r_1_joint = np.empty((self._N,self._N,3))
        self._r_0[:] = np.nan
        self._r_1[:] = np.nan
        self._r_0_joint[:] = np.nan
        self._r_1_joint[:] = np.nan

        # Section unit vectors
        self._u_a = np.zeros((self._N,3))
        self._u_n = np.zeros((self._N,3))
        self._u_s = np.zeros((self._N,3))
        self._u_a_unswept = np.zeros((self._N,3))
        self._u_n_unswept = np.zeros((self._N,3))
        self._u_s_unswept = np.zeros((self._N,3))

        # Control point atmospheric properties
        self._rho = np.zeros(self._N) # Density
        self._nu = np.zeros(self._N) # Viscosity
        self._a = np.ones(self._N) # Speed of sound

        # Airfoil parameters
        self._alpha_approx= np.zeros(self._N)
        self._Re = np.zeros(self._N) # Reynolds number
        self._M = np.zeros(self._N) # Mach number
        self._aL0 = np.zeros(self._N) # Zero-lift angle of attack
        self._CLa = np.zeros(self._N) # Lift slope
        self._am0 = np.zeros(self._N) # Zero-moment angle of attack
        self._esp_f_delta_f = np.zeros(self._N) # Change in effective angle of attack due to flap deflection

        # Velocities
        self._cp_v_inf = np.zeros((self._N,3)) # Control point freestream vector
        self._cp_V_inf = np.zeros(self._N) # Control point freestream magnitude
        self._cp_u_inf = np.zeros((self._N,3)) # Control point freestream unit vector
        self._v_trans = np.zeros((self._num_aircraft,3))
        self._P0_joint_u_inf = np.zeros((self._N,3))
        self._P1_joint_u_inf = np.zeros((self._N,3))

        # Section coefficients
        self._CL = np.zeros(self._N) # Lift coefficient
        self._CD = np.zeros(self._N) # Drag coefficient
        self._Cm = np.zeros(self._N) # Moment coefficient


    def _perform_geometry_and_atmos_calcs(self):
        # Performs calculations necessary for solving NLL which are only dependent on geometry.
        # This speeds up repeated calls to _solve(). This method should be called any time the 
        # geometry is updated, an aircraft is added to the scene, or the position or orientation
        # of an aircraft changes. Note that all calculations occur in the Earth-fixed frame.

        index = 0
        self._airplane_names = []

        # Loop through airplanes
        for airplane_name, airplane_object in self._airplanes.items():

            # Store airplane and segment names to make sure they are always accessed in the same order
            self._airplane_names.append(airplane_name)
            q = airplane_object.q
            p = airplane_object.p_bar

            # Section of the arrays belonging to this airplane
            airplane_N = airplane_object.N
            airplane_slice = slice(index, index+airplane_N)

            # Get geometries
            PC = quat_inv_trans(q, airplane_object.PC)
            self._r_CG[airplane_slice,:] = PC
            self._PC[airplane_slice,:] = p+PC
            self._c_bar[airplane_slice] = airplane_object.c_bar
            self._dS[airplane_slice] = airplane_object.dS
            self._dl[airplane_slice,:] = quat_inv_trans(q, airplane_object.dl)
            self._max_camber[airplane_slice] = airplane_object.max_camber
            self._max_thickness[airplane_slice] = airplane_object.max_thickness
            self._section_sweep[airplane_slice] = airplane_object.section_sweep

            # Get section vectors
            self._u_a[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_a)
            self._u_n[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_n)
            self._u_s[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_s)
            self._u_a_unswept[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_a_unswept)
            self._u_n_unswept[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_n_unswept)
            self._u_s_unswept[airplane_slice,:] = quat_inv_trans(q, airplane_object.u_s_unswept)

            # Node locations
            # Note the first index indicates which control point this is the effective LAC for
            self._P0[airplane_slice,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P0_eff)
            self._P1[airplane_slice,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P1_eff)
            self._P0_joint[airplane_slice,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P0_joint_eff)
            self._P1_joint[airplane_slice,airplane_slice,:] = p+quat_inv_trans(q, airplane_object.P1_joint_eff)

            # Get node locations for other aircraft from this aircraft
            # This does not need to take the effective LAC into account
            this_ind = range(index, index+airplane_N)
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

            # Update position in the arrays
            index += airplane_N

        # Fill in spatial node vectors between airplanes
        self._r_0 = np.where(np.isnan(self._r_0), self._PC[:,np.newaxis,:]-self._P0, self._r_0)
        self._r_1 = np.where(np.isnan(self._r_1), self._PC[:,np.newaxis,:]-self._P1, self._r_1)
        self._r_0_joint = np.where(np.isnan(self._r_0_joint), self._PC[:,np.newaxis,:]-self._P0_joint, self._r_0_joint)
        self._r_1_joint = np.where(np.isnan(self._r_1_joint), self._PC[:,np.newaxis,:]-self._P1_joint, self._r_1_joint)

        # Calculate spatial node vector magnitudes
        self._r_0_mag = np.sqrt(np.einsum('ijk,ijk->ij', self._r_0, self._r_0))
        self._r_0_joint_mag = np.sqrt(np.einsum('ijk,ijk->ij', self._r_0_joint, self._r_0_joint))
        self._r_1_mag = np.sqrt(np.einsum('ijk,ijk->ij', self._r_1, self._r_1))
        self._r_1_joint_mag = np.sqrt(np.einsum('ijk,ijk->ij', self._r_1_joint, self._r_1_joint))

        # Calculate magnitude products
        self._r_0_r_0_joint_mag = self._r_0_mag*self._r_0_joint_mag
        self._r_0_r_1_mag = self._r_0_mag*self._r_1_mag
        self._r_1_r_1_joint_mag = self._r_1_mag*self._r_1_joint_mag

        # Effective freestream projection matrices
        self._P_eff = np.repeat(np.identity(3)[np.newaxis,:,:], self._N, axis=0)-np.matmul(self._u_s[:,:,np.newaxis], self._u_s[:,np.newaxis,:])

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

        # Atmospheric density, speed of sound, and viscosity
        self._rho = self._get_density(self._PC)
        self._a = self._get_sos(self._PC)
        self._nu = self._get_viscosity(self._PC)

        # Swept section corrections
        sweep2 = self._section_sweep**2
        sweep4 = self._section_sweep**4
        tau2 = self._max_thickness*self._max_thickness

        self._delta_a_L0 = 1.0/(1.0+0.5824*self._max_camber**0.92*sweep2+1.3892*self._max_camber**1.16*sweep4)-1
        self._R_CL_a = 1.0/(1.0-0.2955*self._max_thickness**0.96*sweep2-0.1335*self._max_thickness**0.68*sweep4)

        self._delta_a_m0 = 1.0/(1.0+1.07*self._max_camber**0.95*sweep2+0.56*self._max_camber**0.83*sweep4)-1
        self._R_Cm_a = 1.0+(-2.37*self._max_thickness+0.91)*(np.cos((6.62*tau2+1.06)*self._section_sweep)-1.0)

        self._solved = False


    def _calc_invariant_flow_properties(self):
        # Calculates the invariant flow properties at each control point and node location

        # Velocities at vortex nodes
        P0_joint_v_inf = np.zeros((self._N,3))
        P1_joint_v_inf = np.zeros((self._N,3))

        # Get wind velocities at control points and nodes
        cp_v_wind = self._get_wind(self._PC)

        # Loop through airplanes
        index = 0
        for i, airplane_name in enumerate(self._airplane_names):
            airplane_object = self._airplanes[airplane_name]
            N = airplane_object.N
            cur_slice = slice(index, index+N)

            # Determine freestream velocity due to airplane translation
            self._v_trans[i,:] = -airplane_object.v

            # Freestream velocities

            # Control points
            cp_v_rot = quat_inv_trans(airplane_object.q, -np.cross(airplane_object.w, airplane_object.PC))
            self._cp_v_inf[cur_slice,:] = self._v_trans[i,:]+cp_v_wind[cur_slice]+cp_v_rot
            self._cp_V_inf[cur_slice] = np.linalg.norm(self._cp_v_inf[cur_slice,:], axis=1)
            self._cp_u_inf[cur_slice,:] = self._cp_v_inf[cur_slice]/self._cp_V_inf[cur_slice,np.newaxis]

            # P0 joint
            P0_joint_v_rot = quat_inv_trans(airplane_object.q, -np.cross(airplane_object.w, airplane_object.P0_joint))
            P0_joint_v_inf[cur_slice,:] = self._v_trans[i,:]+cp_v_wind[cur_slice]+P0_joint_v_rot

            # P1 joint
            P1_joint_v_rot = quat_inv_trans(airplane_object.q, -np.cross(airplane_object.w, airplane_object.P1_joint))
            P1_joint_v_inf[cur_slice,:] = self._v_trans[i,:]+cp_v_wind[cur_slice]+P1_joint_v_rot

            # Calculate airfoil parameters (Re and M are only used in the linear solution)
            self._alpha_approx[cur_slice] = np.einsum('ij,ij->i', self._cp_u_inf[cur_slice,:], self._u_n[cur_slice,:])
            self._Re[cur_slice] = self._cp_V_inf[cur_slice]*self._c_bar[cur_slice]/self._nu[cur_slice]
            self._M[cur_slice] = self._cp_V_inf[cur_slice]/self._a[cur_slice]

            # Get lift slopes and zero-lift angles of attack for each segment
            seg_ind = 0
            for segment in airplane_object.segments:
                seg_N = segment.N
                seg_slice = slice(index+seg_ind, index+seg_ind+seg_N)
                self._CLa[seg_slice] = segment.get_cp_CLa(self._alpha_approx[seg_slice], self._Re[seg_slice], self._M[seg_slice])
                self._aL0[seg_slice] = segment.get_cp_aL0(self._Re[seg_slice], self._M[seg_slice])
                self._am0[seg_slice] = segment.get_cp_am0(self._Re[seg_slice], self._M[seg_slice])
                self._esp_f_delta_f[seg_slice] = segment.get_cp_flap_eff()
                seg_ind += seg_N

            index += N

        # Calculate nodal freestream unit vectors
        self._P0_joint_u_inf = P0_joint_v_inf/np.linalg.norm(P0_joint_v_inf, axis=-1, keepdims=True)
        self._P1_joint_u_inf = P1_joint_v_inf/np.linalg.norm(P1_joint_v_inf, axis=-1, keepdims=True)

        self._solved = False


    def _calc_V_ji(self):
        # Calculates the influence of each horseshoe vortex on each control point, divided by the vortex strength

        # Influence of vortex segment 0 after the joint; ignore if the radius goes to zero
        denom = (self._r_0_joint_mag*(self._r_0_joint_mag-np.einsum('ijk,ijk->ij', self._P0_joint_u_inf[np.newaxis], self._r_0_joint)))
        V_ji_due_to_0 = np.nan_to_num(-np.cross(self._P0_joint_u_inf, self._r_0_joint)/denom[:,:,np.newaxis], nan=0.0)

        # Influence of vortex segment 1 after the joint
        denom = (self._r_1_joint_mag*(self._r_1_joint_mag-np.einsum('ijk,ijk->ij', self._P1_joint_u_inf[np.newaxis], self._r_1_joint)))
        V_ji_due_to_1 = np.nan_to_num(np.cross(self._P1_joint_u_inf, self._r_1_joint)/denom[:,:,np.newaxis], nan=0.0)

        # Sum and transpose
        self._V_ji = 1/(4*np.pi)*(V_ji_due_to_0+self._V_ji_const+V_ji_due_to_1)


    def _solve_w_scipy(self, **kwargs):
        # Determines the votrex strengths using scipy.fsolve

        # Initialize
        verbose = kwargs.get("verbose", False)
        if verbose: print("Running scipy solver...")
        start_time = time.time()

        # Set up flow for what won't change with changes in vorticity distribution
        self._calc_invariant_flow_properties()
        self._calc_V_ji()

        # Initial guess
        gamma_init = np.zeros(self._N)

        # Get solution
        self._gamma, info, ier, mesg = sopt.fsolve(self._lifting_line_residual, gamma_init, full_output=True, xtol=self._solver_convergence)

        # Output fsolve info
        if verbose:
            print("Complete!")
            print("   Number of function calls: {0}".format(info["nfev"]))
            print("   Norm of final residual vector: {0}".format(np.linalg.norm(info["fvec"])))

        # Check for no solution
        if ier != 1:
            print("Scipy.optimize.fsolve was unable to find a solution.")
            print("Error message: {0}".format(mesg))
            warnings.warn("Scipy solver failed. Reverting to nonlinear solution...")
            return -1

        end_time = time.time()
        return end_time-start_time


    def _lifting_line_residual(self, gamma):
        # Returns the residual to nonlinear lifting-line equation

        # Set vorticity
        self._gamma = gamma

        # Calculate control point velocities
        self._calc_v_i()

        # Get vortex lift
        self._w_i = np.cross(self._v_i, self._dl)
        self._w_i_mag = np.sqrt(np.einsum('ij,ij->i', self._w_i, self._w_i))
        L_vortex = self._w_i_mag*self._gamma
        
        # Get section lift
        L_section = self._get_section_lift()

        # Return difference
        return L_vortex-L_section


    def _calc_v_i(self):
        # Determines the local velocity at each control point
        self._v_i = self._cp_v_inf+(self._V_ji.transpose((2,0,1))@self._gamma).T

    
    def _get_section_lift(self):
        # Calculate magnitude of lift due to section properties divided by density

        # Project velocity into effective airfoil section plane
        self._v_i_eff = np.matmul(self._P_eff, self._v_i[:,:,np.newaxis]).reshape((self._N,3))
        self._V_eff_2 = np.einsum('ij,ij->i', self._v_i_eff, self._v_i_eff)
        self._V_eff = np.sqrt(self._V_eff_2)

        # Calculate swept airfoil parameters
        self._v_a = np.einsum('ij,ij->i', self._v_i_eff, self._u_a)
        self._v_n = np.einsum('ij,ij->i', self._v_i_eff, self._u_n)
        #v_s = np.einsum('ij,ij->i', self._v_i-self._v_i_eff, self._u_s)
        self._alpha_swept = np.arctan2(self._v_n, self._v_a)
        #self._beta_swept = np.arctan2(v_s, self._v_a)

        # Calculate lift
        self._Re = self._V_eff*self._c_bar/self._nu
        self._M = self._V_eff/self._a
        self._CL = np.zeros(self._N)
        index = 0

        # Loop through airplanes
        for airplane_name in self._airplane_names:
            airplane_object = self._airplanes[airplane_name]
            N = airplane_object.N
            seg_ind = 0

            # Loop through segments
            for segment in airplane_object.segments:
                seg_N = segment.N
                seg_slice = slice(index+seg_ind, index+seg_ind+seg_N)
                self._CL[seg_slice] = segment.get_cp_CL(self._alpha_swept[seg_slice], self._Re[seg_slice], self._M[seg_slice])
                seg_ind += seg_N

            index += N

        # Correct swept section lift
        self._correct_CL_for_sweep()

        # Determine Jackson's dimensionalization correction factors
        #V_i_2 = np.einsum('ij,ij->i', self._v_i, self._v_i)
        #R_i = np.sqrt(self._V_eff_2/V_i_2)
        #S_a = np.sin(self._alpha_swept)
        #S_B = np.sin(self._beta_swept)
        #R_i_lambda = np.cos(self._beta_swept)/np.sqrt(1-S_a*S_a*S_B*S_B)

        #V_inf_2 = np.einsum('ij,ij->i', self._cp_v_inf, self._cp_v_inf)
        #return 0.5*V_inf_2*R_i_lambda*R_i*self._CL*self._dS # Jackson's definition

        return 0.5*self._V_eff_2*self._CL*self._dS # Mine, which is so much cleaner...


    def _correct_CL_for_sweep(self):
        # Applies Jackson's corrections for swept section lift

        ## Estimate lift slope
        CL_a_est = self._CL/(self._alpha_swept-self._aL0)

        # Get new estimate
        self._CLa = self._R_CL_a*CL_a_est
        self._aL0 = self._aL0+self._delta_a_L0
        self._CL = self._CLa*(self._alpha_swept-self._aL0)


    def _correct_Cm_for_sweep(self):
        # Applies Jackson's corrections for swept section moment

        ## Estimate lift slope
        Cm_a_est = self._Cm/(self._alpha_swept-self._am0)

        # Get new estimate
        self._Cma = self._R_Cm_a*Cm_a_est
        self._am0 = self._am0+self._delta_a_m0
        self._Cm = np.where(self._Cm != 0.0, self._Cma*(self._alpha_swept-self._am0), 0.0)


    def _solve_linear(self, **kwargs):
        # Determines the vortex strengths of all horseshoe vortices in the scene using the linearize equations

        verbose = kwargs.get("verbose", False)
        if verbose: print("Running linear solver...")
        start_time = time.time()

        # Calculate invariant properties
        self._calc_invariant_flow_properties()

        # Calculate V_ji
        self._calc_V_ji()

        # A matrix
        A = np.zeros((self._N,self._N))
        V_ji_dot_u_n = np.einsum('ijk,ijk->ij', self._V_ji, self._u_n[:,np.newaxis])
        A[:,:] = -(self._R_CL_a*self._CLa*self._dS)[:,np.newaxis]*V_ji_dot_u_n
        diag_ind = np.diag_indices(self._N)
        u_inf_x_dl = np.cross(self._cp_u_inf, self._dl)
        A[diag_ind] += 2*np.sqrt(np.einsum('ij,ij->i', u_inf_x_dl, u_inf_x_dl))

        # b vector
        b = self._cp_V_inf*self._CLa*self._dS*(self._alpha_approx-self._aL0+self._esp_f_delta_f-self._delta_a_L0)

        # Solve
        self._gamma = np.linalg.solve(A, b)

        end_time = time.time()
        return end_time-start_time


    def _solve_nonlinear(self, **kwargs):
        # Nonlinear improvement to the vector of gammas already determined
        verbose = kwargs.get("verbose", False)
        if verbose: 
            print("Running nonlinear solver...")
            print("    Relaxation: {0}".format(self._solver_relaxation))
            print("    Convergence: {0}".format(self._solver_convergence))
            print("{0:<20}{1:<20}".format("Iteration", "Error"))
            print("".join(['-']*40))
        start_time = time.time()

        # This parameter, if set to true, will revert the nonlinear solution to a dimensional version of Phillips' original Jacobian.
        # The other way is my new (better) way.
        phillips = False

        J = np.zeros((self._N, self._N))

        # Airfoil coefs
        C_LRe = np.zeros(self._N)
        C_LM = np.zeros(self._N)

        iteration = 0
        error = 100
        while error > self._solver_convergence:
            iteration += 1

            # Get residual vector
            R = self._lifting_line_residual(self._gamma)
            error = np.linalg.norm(R)

            # Calculate airfoil parameters
            self._v_a = np.einsum('ij,ij->i', self._v_i_eff, self._u_a)
            self._v_n = np.einsum('ij,ij->i', self._v_i_eff, self._u_n)
            self._alpha_swept = np.arctan2(self._v_n, self._v_a)

            index = 0

            # Loop through airplanes
            for airplane_name in self._airplane_names:
                airplane_object = self._airplanes[airplane_name]

                # Loop through segments
                for segment_object in airplane_object.segments:
                    num_cps = segment_object.N
                    cur_slice = slice(index, index+num_cps)

                    # Get lift coefficient and lift slopes
                    self._CLa[cur_slice] = segment_object.get_cp_CLa(self._alpha_swept[cur_slice], self._Re[cur_slice], self._M[cur_slice])
                    C_LRe[cur_slice] = segment_object.get_cp_CLRe(self._alpha_swept[cur_slice], self._Re[cur_slice], self._M[cur_slice])
                    C_LM[cur_slice] = segment_object.get_cp_CLM(self._alpha_swept[cur_slice], self._Re[cur_slice], self._M[cur_slice])

                    index += num_cps

            # Intermediate calcs
            v_iji = np.einsum('ijk,ijk->ij', self._v_i[:,np.newaxis,:], self._V_ji)

            # Caclulate Jacobian
            J[:,:] = (2*self._gamma/self._w_i_mag)[:,np.newaxis]*(np.einsum('ijk,ijk->ij', self._w_i[:,np.newaxis,:], np.cross(self._V_ji, self._dl)))

            if not phillips:
                J[:,:] -= (2*self._dS*self._CL)[:,np.newaxis]*v_iji # Comes from taking the derivative of V_i^2 with respect to gamma

            CL_gamma_alpha = self._CLa[:,np.newaxis]*(self._v_a[:,np.newaxis]*np.einsum('ijk,ijk->ij', self._V_ji, self._u_n[:,np.newaxis])-self._v_n[:,np.newaxis]*np.einsum('ijk,ijk->ij', self._V_ji, self._u_a[:,np.newaxis]))/(self._v_n*self._v_n+self._v_a*self._v_a)[:,np.newaxis]
            CL_gamma_Re = C_LRe[:,np.newaxis]*self._c_bar/(self._nu*self._V_eff)[:,np.newaxis]*v_iji
            CL_gamma_M = C_LM[:,np.newaxis]/(self._a*self._V_eff)[:,np.newaxis]*v_iji

            if phillips:
                J[:,:] -= (self._cp_V_inf*self._cp_V_inf*self._dS)[:,np.newaxis]*(CL_gamma_alpha) # Phillips' way
            else:
                J[:,:] -= (self._V_eff_2*self._dS)[:,np.newaxis]*(CL_gamma_alpha+CL_gamma_Re+CL_gamma_M) # My way

            diag_ind = np.diag_indices(self._N)
            J[diag_ind] += 2*self._w_i_mag

            # Solve for change in gamma
            dGamma = np.linalg.solve(J, -R)

            # Update gammas
            self._gamma = self._gamma+self._solver_relaxation*dGamma

            # Output progress
            if verbose: print("{0:<20}{1:<20}".format(iteration, error))

            # Check this isn't taking too long
            if iteration >= self._max_solver_iterations:
                if verbose: print("Nonlinear solver failed to converge within the allowed number of iterations. Final error: {0}".format(error))
                break

        else: # If the loop exits normally, then everything is good
            if verbose: print("Nonlinear solver successfully converged.")

        end_time = time.time()
        return end_time-start_time


    def _integrate_forces_and_moments(self, **kwargs):
        # Determines the forces and moments on each lifting surface
        start_time = time.time()

        # Kwargs
        non_dimensional = kwargs.get("non_dimensional", True)
        dimensional = kwargs.get("dimensional", True)
        report_by_segment = kwargs.get("report_by_segment", False)

        # Get velocities
        self._calc_v_i()
        self._V_i_2 = np.einsum('ij,ij->i', self._v_i, self._v_i)
        self._V_i = np.sqrt(self._V_i_2)
        self._u_i = self._v_i/self._V_i[:,np.newaxis]

        # Calculate vortex force differential elements
        dF_inv = (self._rho*self._gamma)[:,np.newaxis]*np.cross(self._v_i, self._dl)
        self._dL = np.linalg.norm(dF_inv, axis=-1)

        # Calculate conditions for determining viscid contributions
        if not hasattr(self, "_alpha_swept"):
            self._v_i_eff = np.matmul(self._P_eff, self._v_i[:,:,np.newaxis]).reshape((self._N,3))
            self._v_a = np.einsum('ij,ij->i', self._v_i_eff, self._u_a)
            self._v_n = np.einsum('ij,ij->i', self._v_i_eff, self._u_n)
            self._alpha_swept = np.arctan2(self._v_n, self._v_a)
        self._q_i = 0.5*self._rho*self._V_i_2
        self._redim = self._q_i*self._dS

        # Store lift, drag, and moment coefficient distributions
        self._FM = {}
        empty_coef_dict = { "CL" : {}, "CD" : {}, "CS" : {}, "Cx" : {}, "Cy" : {}, "Cz" : {}, "Cl" : {}, "Cm" : {}, "Cn" : {}}
        empty_FM_dict = { "FL" : {}, "FD" : {}, "FS" : {}, "Fx" : {}, "Fy" : {}, "Fz" : {}, "Mx" : {}, "My" : {}, "Mz" : {}}

        # Get section moment and drag coefficients
        index = 0
        for airplane_name in self._airplane_names:
            for segment in self._airplanes[airplane_name].segments:
                num_cps = segment.N
                cur_slice = slice(index, index+num_cps)

                # Section drag
                self._CD[cur_slice] = segment.get_cp_CD(self._alpha_swept[cur_slice], self._Re[cur_slice], self._M[cur_slice])

                # Determine moment due to section moment coef
                self._Cm[cur_slice] = segment.get_cp_Cm(self._alpha_swept[cur_slice], self._Re[cur_slice], self._M[cur_slice])

                index += num_cps

        # Make sweep corrections
        C_sweep_inv = 1.0/np.cos(self._section_sweep)
        self._correct_Cm_for_sweep()
        self._CD *= C_sweep_inv

        # Determine viscous drag vector
        dD = self._redim*self._CD
        dF_b_visc = dD[:,np.newaxis]*self._u_i

        # Moment due to viscous drag
        dM_visc = np.cross(self._r_CG, dF_b_visc)

        # Inviscid moment
        dM_vortex = np.cross(self._r_CG, dF_inv)
        dM_section = -(self._redim*self._c_bar*C_sweep_inv*self._Cm)[:,np.newaxis]*self._u_s
        dM_inv = dM_vortex+dM_section

        # Loop through airplanes to gather necessary data
        index = 0
        for i, airplane_name in enumerate(self._airplane_names):
            airplane_object = self._airplanes[airplane_name]
            FM_inv_airplane_total = np.zeros(9)
            FM_vis_airplane_total = np.zeros(9)

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
            v_inf = self._v_trans[i,:] + self._get_wind(airplane_object.p_bar)
            V_inf = np.linalg.norm(v_inf)
            u_inf = quat_trans(airplane_object.q, (v_inf/V_inf).flatten())

            # Determine reference parameters
            if non_dimensional:
                S_w =  airplane_object.S_w
                l_ref_lon = airplane_object.l_ref_lon
                l_ref_lat = airplane_object.l_ref_lat
                rho_ref = self._get_density(airplane_object.p_bar)
                q_ref = 0.5*rho_ref*V_inf*V_inf

            # Loop through segments
            for segment in airplane_object.segments:
                num_cps = segment.N
                segment_name = segment.name
                cur_slice = slice(index, index+num_cps)

                # Get drag coef and redimensionalize
                F_b_visc = quat_trans(airplane_object.q, np.sum(dF_b_visc[cur_slice], axis=0))
                L_visc, D_visc, S_visc = self._rotate_aero_forces(F_b_visc, u_inf)

                # Determine viscous moment vector
                M_b_visc = quat_trans(airplane_object.q, np.sum(dM_visc[cur_slice], axis=0))

                # Determine inviscid force vector
                F_b_inv = quat_trans(airplane_object.q, np.sum(dF_inv[cur_slice], axis=0))
                L_inv, D_inv, S_inv = self._rotate_aero_forces(F_b_inv, u_inf)

                # Combine moment due to lift and section moment
                M_b_inv = quat_trans(airplane_object.q, np.sum(dM_inv[cur_slice], axis=0))

                # Store
                if report_by_segment:
                    if non_dimensional:
                        self._FM[airplane_name]["viscous"]["Cx"][segment_name] = F_b_visc[0].item()/(q_ref*S_w)
                        self._FM[airplane_name]["viscous"]["Cy"][segment_name] = F_b_visc[1].item()/(q_ref*S_w)
                        self._FM[airplane_name]["viscous"]["Cz"][segment_name] = F_b_visc[2].item()/(q_ref*S_w)

                        self._FM[airplane_name]["viscous"]["CL"][segment_name] = L_visc/(q_ref*S_w)
                        self._FM[airplane_name]["viscous"]["CD"][segment_name] = D_visc/(q_ref*S_w)
                        self._FM[airplane_name]["viscous"]["CS"][segment_name] = S_visc/(q_ref*S_w)

                        self._FM[airplane_name]["viscous"]["Cl"][segment_name] = M_b_visc[0].item()/(q_ref*S_w*l_ref_lat)
                        self._FM[airplane_name]["viscous"]["Cm"][segment_name] = M_b_visc[1].item()/(q_ref*S_w*l_ref_lon)
                        self._FM[airplane_name]["viscous"]["Cn"][segment_name] = M_b_visc[2].item()/(q_ref*S_w*l_ref_lat)

                        self._FM[airplane_name]["inviscid"]["Cx"][segment_name] = F_b_inv[0].item()/(q_ref*S_w)
                        self._FM[airplane_name]["inviscid"]["Cy"][segment_name] = F_b_inv[1].item()/(q_ref*S_w)
                        self._FM[airplane_name]["inviscid"]["Cz"][segment_name] = F_b_inv[2].item()/(q_ref*S_w)

                        self._FM[airplane_name]["inviscid"]["CL"][segment_name] = L_inv/(q_ref*S_w)
                        self._FM[airplane_name]["inviscid"]["CD"][segment_name] = D_inv/(q_ref*S_w)
                        self._FM[airplane_name]["inviscid"]["CS"][segment_name] = S_inv/(q_ref*S_w)

                        self._FM[airplane_name]["inviscid"]["Cl"][segment_name] = M_b_inv[0].item()/(q_ref*S_w*l_ref_lat)
                        self._FM[airplane_name]["inviscid"]["Cm"][segment_name] = M_b_inv[1].item()/(q_ref*S_w*l_ref_lon)
                        self._FM[airplane_name]["inviscid"]["Cn"][segment_name] = M_b_inv[2].item()/(q_ref*S_w*l_ref_lat)

                    if dimensional:
                        self._FM[airplane_name]["viscous"]["Fx"][segment_name] = F_b_visc[0].item()
                        self._FM[airplane_name]["viscous"]["Fy"][segment_name] = F_b_visc[1].item()
                        self._FM[airplane_name]["viscous"]["Fz"][segment_name] = F_b_visc[2].item()

                        self._FM[airplane_name]["viscous"]["FL"][segment_name] = L_visc
                        self._FM[airplane_name]["viscous"]["FD"][segment_name] = D_visc
                        self._FM[airplane_name]["viscous"]["FS"][segment_name] = S_visc

                        self._FM[airplane_name]["viscous"]["Mx"][segment_name] = M_b_visc[0].item()
                        self._FM[airplane_name]["viscous"]["My"][segment_name] = M_b_visc[1].item()
                        self._FM[airplane_name]["viscous"]["Mz"][segment_name] = M_b_visc[2].item()

                        self._FM[airplane_name]["inviscid"]["Fx"][segment_name] = F_b_inv[0].item()
                        self._FM[airplane_name]["inviscid"]["Fy"][segment_name] = F_b_inv[1].item()
                        self._FM[airplane_name]["inviscid"]["Fz"][segment_name] = F_b_inv[2].item()

                        self._FM[airplane_name]["inviscid"]["FL"][segment_name] = L_inv
                        self._FM[airplane_name]["inviscid"]["FD"][segment_name] = D_inv
                        self._FM[airplane_name]["inviscid"]["FS"][segment_name] = S_inv

                        self._FM[airplane_name]["inviscid"]["Mx"][segment_name] = M_b_inv[0].item()
                        self._FM[airplane_name]["inviscid"]["My"][segment_name] = M_b_inv[1].item()
                        self._FM[airplane_name]["inviscid"]["Mz"][segment_name] = M_b_inv[2].item()

                # Sum up totals
                FM_inv_airplane_total[0] += L_inv
                FM_inv_airplane_total[1] += D_inv
                FM_inv_airplane_total[2] += S_inv
                FM_vis_airplane_total[0] += L_visc
                FM_vis_airplane_total[1] += D_visc
                FM_vis_airplane_total[2] += S_visc
                FM_inv_airplane_total[3:6] += F_b_inv
                FM_vis_airplane_total[3:6] += F_b_visc
                FM_inv_airplane_total[6:] += M_b_inv
                FM_vis_airplane_total[6:] += M_b_visc
                
                index += num_cps

            if non_dimensional:
                # Store the total inviscid force and moment
                self._FM[airplane_name]["inviscid"]["CL"]["total"] = FM_inv_airplane_total[0].item()/(q_ref*S_w)
                self._FM[airplane_name]["inviscid"]["CD"]["total"] = FM_inv_airplane_total[1].item()/(q_ref*S_w)
                self._FM[airplane_name]["inviscid"]["CS"]["total"] = FM_inv_airplane_total[2].item()/(q_ref*S_w)
                self._FM[airplane_name]["inviscid"]["Cx"]["total"] = FM_inv_airplane_total[3].item()/(q_ref*S_w)
                self._FM[airplane_name]["inviscid"]["Cy"]["total"] = FM_inv_airplane_total[4].item()/(q_ref*S_w)
                self._FM[airplane_name]["inviscid"]["Cz"]["total"] = FM_inv_airplane_total[5].item()/(q_ref*S_w)
                self._FM[airplane_name]["inviscid"]["Cl"]["total"] = FM_inv_airplane_total[6].item()/(q_ref*S_w*l_ref_lat)
                self._FM[airplane_name]["inviscid"]["Cm"]["total"] = FM_inv_airplane_total[7].item()/(q_ref*S_w*l_ref_lon)
                self._FM[airplane_name]["inviscid"]["Cn"]["total"] = FM_inv_airplane_total[8].item()/(q_ref*S_w*l_ref_lat)

                # Store the total viscous force and moment
                self._FM[airplane_name]["viscous"]["CL"]["total"] = FM_vis_airplane_total[0].item()/(q_ref*S_w)
                self._FM[airplane_name]["viscous"]["CD"]["total"] = FM_vis_airplane_total[1].item()/(q_ref*S_w)
                self._FM[airplane_name]["viscous"]["CS"]["total"] = FM_vis_airplane_total[2].item()/(q_ref*S_w)
                self._FM[airplane_name]["viscous"]["Cx"]["total"] = FM_vis_airplane_total[3].item()/(q_ref*S_w)
                self._FM[airplane_name]["viscous"]["Cy"]["total"] = FM_vis_airplane_total[4].item()/(q_ref*S_w)
                self._FM[airplane_name]["viscous"]["Cz"]["total"] = FM_vis_airplane_total[5].item()/(q_ref*S_w)
                self._FM[airplane_name]["viscous"]["Cl"]["total"] = FM_vis_airplane_total[6].item()/(q_ref*S_w*l_ref_lat)
                self._FM[airplane_name]["viscous"]["Cm"]["total"] = FM_vis_airplane_total[7].item()/(q_ref*S_w*l_ref_lon)
                self._FM[airplane_name]["viscous"]["Cn"]["total"] = FM_vis_airplane_total[8].item()/(q_ref*S_w*l_ref_lat)

                # Determine total force and moment for the airplane
                FM_airplane_total = FM_vis_airplane_total+FM_inv_airplane_total
                self._FM[airplane_name]["total"]["CL"] = FM_airplane_total[0].item()/(q_ref*S_w)
                self._FM[airplane_name]["total"]["CD"] = FM_airplane_total[1].item()/(q_ref*S_w)
                self._FM[airplane_name]["total"]["CS"] = FM_airplane_total[2].item()/(q_ref*S_w)
                self._FM[airplane_name]["total"]["Cx"] = FM_airplane_total[3].item()/(q_ref*S_w)
                self._FM[airplane_name]["total"]["Cy"] = FM_airplane_total[4].item()/(q_ref*S_w)
                self._FM[airplane_name]["total"]["Cz"] = FM_airplane_total[5].item()/(q_ref*S_w)
                self._FM[airplane_name]["total"]["Cl"] = FM_airplane_total[6].item()/(q_ref*S_w*l_ref_lat)
                self._FM[airplane_name]["total"]["Cm"] = FM_airplane_total[7].item()/(q_ref*S_w*l_ref_lon)
                self._FM[airplane_name]["total"]["Cn"] = FM_airplane_total[8].item()/(q_ref*S_w*l_ref_lat)

            if dimensional:
                # Store the total inviscid force and moment
                self._FM[airplane_name]["inviscid"]["FL"]["total"] = FM_inv_airplane_total[0].item()
                self._FM[airplane_name]["inviscid"]["FD"]["total"] = FM_inv_airplane_total[1].item()
                self._FM[airplane_name]["inviscid"]["FS"]["total"] = FM_inv_airplane_total[2].item()
                self._FM[airplane_name]["inviscid"]["Fx"]["total"] = FM_inv_airplane_total[3].item()
                self._FM[airplane_name]["inviscid"]["Fy"]["total"] = FM_inv_airplane_total[4].item()
                self._FM[airplane_name]["inviscid"]["Fz"]["total"] = FM_inv_airplane_total[5].item()
                self._FM[airplane_name]["inviscid"]["Mx"]["total"] = FM_inv_airplane_total[6].item()
                self._FM[airplane_name]["inviscid"]["My"]["total"] = FM_inv_airplane_total[7].item()
                self._FM[airplane_name]["inviscid"]["Mz"]["total"] = FM_inv_airplane_total[8].item()

                # Store the total viscous force and moment
                self._FM[airplane_name]["viscous"]["FL"]["total"] = FM_vis_airplane_total[0].item()
                self._FM[airplane_name]["viscous"]["FD"]["total"] = FM_vis_airplane_total[1].item()
                self._FM[airplane_name]["viscous"]["FS"]["total"] = FM_vis_airplane_total[2].item()
                self._FM[airplane_name]["viscous"]["Fx"]["total"] = FM_vis_airplane_total[3].item()
                self._FM[airplane_name]["viscous"]["Fy"]["total"] = FM_vis_airplane_total[4].item()
                self._FM[airplane_name]["viscous"]["Fz"]["total"] = FM_vis_airplane_total[5].item()
                self._FM[airplane_name]["viscous"]["Mx"]["total"] = FM_vis_airplane_total[6].item()
                self._FM[airplane_name]["viscous"]["My"]["total"] = FM_vis_airplane_total[7].item()
                self._FM[airplane_name]["viscous"]["Mz"]["total"] = FM_vis_airplane_total[8].item()

                # Determine total force and moment for the airplane
                FM_airplane_total = FM_vis_airplane_total+FM_inv_airplane_total
                self._FM[airplane_name]["total"]["FL"] = FM_airplane_total[0].item()
                self._FM[airplane_name]["total"]["FD"] = FM_airplane_total[1].item()
                self._FM[airplane_name]["total"]["FS"] = FM_airplane_total[2].item()
                self._FM[airplane_name]["total"]["Fx"] = FM_airplane_total[3].item()
                self._FM[airplane_name]["total"]["Fy"] = FM_airplane_total[4].item()
                self._FM[airplane_name]["total"]["Fz"] = FM_airplane_total[5].item()
                self._FM[airplane_name]["total"]["Mx"] = FM_airplane_total[6].item()
                self._FM[airplane_name]["total"]["My"] = FM_airplane_total[7].item()
                self._FM[airplane_name]["total"]["Mz"] = FM_airplane_total[8].item()

        end_time = time.time()
        return end_time-start_time


    def _rotate_aero_forces(self, F, u_inf):
        # Takes the body-fixed force vector and coverts it to lift, drag, and sideforce
        # This uses the AeroLab convention where beta is asin(Vy/V)

        # Determine direction vectors
        u_lift = np.cross(u_inf, [0.,1.,0.])
        u_lift = u_lift/np.linalg.norm(u_lift)
        u_side = np.cross(u_lift, u_inf)
        u_side = u_side/np.linalg.norm(u_side)

        # Drag force
        D = np.dot(F, u_inf)
        D_vec = D*u_inf
        
        # Lift force
        L = np.dot(F, u_lift)
        L_vec = L*u_lift
        
        # Side force
        S_vec = F-L_vec-D_vec
        S = np.dot(S_vec,u_side)

        return L,D,S


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

        verbose : bool
            Display the time it took to complete each portion of the calculation. 
            Defaults to False.

        Returns
        -------
        dict:
            Dictionary of forces and moments acting on each wing segment.
        """

        # Check for aircraft
        if self._num_aircraft == 0:
            raise RuntimeError("There are no aircraft in this scene. No calculations can be performed.")

        # Solve for gamma distribution
        fsolve_time = 0.0
        if self._solver_type == "scipy_fsolve":
            fsolve_time = self._solve_w_scipy(**kwargs)

        linear_time = 0.0
        nonlinear_time = 0.0
        if self._solver_type != "scipy_fsolve" or fsolve_time == -1:

            if fsolve_time == -1:
                fsolve_time = 0.0

            # Linear solution
            linear_time = self._solve_linear(**kwargs)

            # Nonlinear improvement
            if self._solver_type == "nonlinear":
                nonlinear_time = self._solve_nonlinear(**kwargs)

        # Integrate forces and moments
        integrate_time = self._integrate_forces_and_moments(**kwargs)

        # Output timing
        verbose = kwargs.get("verbose", False)
        if verbose:
            print("Time to compute circulation distribution using scipy.fsolve: {0} s".format(fsolve_time))
            print("Time to compute circulation distribution using linear equations: {0} s".format(linear_time))
            print("Time to compute nonlinear improvement to circulation distribution: {0} s".format(nonlinear_time))
            total_time = linear_time+nonlinear_time+integrate_time+fsolve_time
            print("Time to integrate forces: {0} s".format(integrate_time))
            print("Total time: {0} s".format(total_time))
            print("Solution rate: {0} Hz".format(1/total_time))

        # Output to file
        filename = kwargs.get("filename", None)
        if filename is not None:
            with open(filename, 'w') as json_file_handle:
                json.dump(self._FM, json_file_handle, indent=4)

        # Let certain functions know the results are now available
        self._solved = True

        return self._FM


    def set_aircraft_state(self, state={}, aircraft_name=None):
        """Sets the state of the given aircraft.

        Parameters
        ----------
        state : dict
            Dictionary describing the state as specified in 
            'Creating Input Files for MachUp'. The state type may be 
            aerodynamic or rigid-body, regardless of how the 
            state was originally specified. Any values not given 
            default to their original defaults. The previous 
            state of the aircraft is in no way preserved.

        aircraft_name : str
            The name of the aircraft to set the state of. If there
            is only one aircraft in the scene, this does not need 
            to be specified.
        """

        # Specify the only aircraft if not already specified
        if aircraft_name is None:
            if self._num_aircraft == 1:
                aircraft_name = list(self._airplanes.keys())[0]
            else:
                raise IOError("Aircraft name must be specified if there is more than one aircraft in the scene.")

        # Determine wind velocity
        aircraft_position = state.get("position", np.array([0,0,0]))
        v_wind = self._get_wind(aircraft_position)

        # Set state and update precalcs for NLL
        old_position = self._airplanes[aircraft_name].p_bar
        old_orient = self._airplanes[aircraft_name].q
        self._airplanes[aircraft_name].set_state(**state, v_wind=v_wind)
        aircraft_orient = self._airplanes[aircraft_name].q

        # If the position has changed, then we need to update the geometry
        if not np.allclose(old_position, aircraft_position) or not np.allclose(old_orient, aircraft_orient):
            self._perform_geometry_and_atmos_calcs()


    def set_aircraft_control_state(self, control_state={}, aircraft_name=None):
        """Sets the control state of the given aircraft.

        Parameters
        ----------
        control_state : dict
            Dictionary describing the control state. Each key value pair should be
            the name of the control and its deflection in degrees.

        aircraft_name : str
            The name of the aircraft to set the state of. If there
            is only one aircraft in the scene, this does not need 
            to be specified.
        """

        # Specify the only aircraft if not already specified
        if aircraft_name is None:
            if self._num_aircraft == 1:
                aircraft_name = list(self._airplanes.keys())[0]
            else:
                raise IOError("Aircraft name must be specified if there is more than one aircraft in the scene.")

        # Set state
        self._airplanes[aircraft_name].set_control_state(control_state)
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
                points = airplane_object.p_bar+quat_inv_trans(airplane_object.q, segment_object.get_outline_points())

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


    def aircraft_derivatives(self, **kwargs):
        """Determines the stability, damping, and control derivatives at the 
        current state. Uses a central difference sceme.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the aerodynamic derivatives 
            of. Defaults to all aircraft in the scene.

        filename : str
            File to export the results to. Defaults to no file.

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
            derivs[aircraft_name]["stability"] = self.aircraft_stability_derivatives(aircraft=aircraft_name)[aircraft_name]
        
            # Determine damping derivatives
            derivs[aircraft_name]["damping"] = self.aircraft_damping_derivatives(aircraft=aircraft_name)[aircraft_name]

            # Determine control derivatives
            derivs[aircraft_name]["control"] = self.aircraft_control_derivatives(aircraft=aircraft_name)[aircraft_name]

        # Export to file
        filename = kwargs.get("filename", None)
        if filename is not None:
            with open(filename, 'w') as output_handle:
                json.dump(derivs, output_handle, indent=4)

        return derivs

    def aircraft_stability_derivatives(self, aircraft=None, dtheta=0.5):
        """Determines the stability derivatives at the current state. Uses 
        a central difference sceme.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the aerodynamic derivatives 
            of. Defaults to all aircraft in the scene.

        dtheta : float
            The finite difference in degrees used to perturb alpha and beta
            and determine the derivatives. Defaults to 0.5

        Returns
        -------
        dict
            A dictionary of stability derivatives.
        """
        derivs= {}

        # Specify the aircraft
        if aircraft is None:
            aircraft_names = list(self._airplanes.keys())
        elif isinstance(aircraft, list):
            aircraft_names = copy.copy(aircraft)
        elif isinstance(aircraft, str):
            aircraft_names = [aircraft]
        else:
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft))

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}
            # Get current aerodynamic state
            alpha_0, beta_0,_ = self._airplanes[aircraft_name].get_aerodynamic_state()

            # Perturb forward in alpha
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0+dtheta)
            self.solve_forces(dimensional=False)
            FM_dalpha_fwd = self._FM

            # Perturb backward in alpha
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0-dtheta)
            self.solve_forces(dimensional=False)
            FM_dalpha_bwd = self._FM

            # Perturb forward in beta
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0, beta=beta_0+dtheta) # We have to reset alpha on this one
            self.solve_forces(dimensional=False)
            FM_dbeta_fwd = self._FM

            # Perturb backward in beta
            self._airplanes[aircraft_name].set_aerodynamic_state(beta=beta_0-dtheta)
            self.solve_forces(dimensional=False)
            FM_dbeta_bwd = self._FM

            # Derivatives with respect to alpha
            diff = 1/(2*np.radians(dtheta)) # The derivative is in radians

            derivs[aircraft_name]["CL,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CL"]-FM_dalpha_bwd[aircraft_name]["total"]["CL"])*diff
            derivs[aircraft_name]["CD,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CD"]-FM_dalpha_bwd[aircraft_name]["total"]["CD"])*diff
            derivs[aircraft_name]["CS,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CS"]-FM_dalpha_bwd[aircraft_name]["total"]["CS"])*diff
            derivs[aircraft_name]["Cx,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cx"]-FM_dalpha_bwd[aircraft_name]["total"]["Cx"])*diff
            derivs[aircraft_name]["Cy,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cy"]-FM_dalpha_bwd[aircraft_name]["total"]["Cy"])*diff
            derivs[aircraft_name]["Cz,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cz"]-FM_dalpha_bwd[aircraft_name]["total"]["Cz"])*diff
            derivs[aircraft_name]["Cl,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cl"]-FM_dalpha_bwd[aircraft_name]["total"]["Cl"])*diff
            derivs[aircraft_name]["Cm,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cm"]-FM_dalpha_bwd[aircraft_name]["total"]["Cm"])*diff
            derivs[aircraft_name]["Cn,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cn"]-FM_dalpha_bwd[aircraft_name]["total"]["Cn"])*diff

            # Derivatives with respect to beta
            derivs[aircraft_name]["CL,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["CL"]-FM_dbeta_bwd[aircraft_name]["total"]["CL"])*diff
            derivs[aircraft_name]["CD,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["CD"]-FM_dbeta_bwd[aircraft_name]["total"]["CD"])*diff
            derivs[aircraft_name]["CS,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["CS"]-FM_dbeta_bwd[aircraft_name]["total"]["CS"])*diff
            derivs[aircraft_name]["Cx,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cx"]-FM_dbeta_bwd[aircraft_name]["total"]["Cx"])*diff
            derivs[aircraft_name]["Cy,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cy"]-FM_dbeta_bwd[aircraft_name]["total"]["Cy"])*diff
            derivs[aircraft_name]["Cz,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cz"]-FM_dbeta_bwd[aircraft_name]["total"]["Cz"])*diff
            derivs[aircraft_name]["Cl,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cl"]-FM_dbeta_bwd[aircraft_name]["total"]["Cl"])*diff
            derivs[aircraft_name]["Cm,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cm"]-FM_dbeta_bwd[aircraft_name]["total"]["Cm"])*diff
            derivs[aircraft_name]["Cn,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cn"]-FM_dbeta_bwd[aircraft_name]["total"]["Cn"])*diff
        
            # Reset aerodynamic state
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0, beta=beta_0)
            self._solved = False

        return derivs


    def aircraft_damping_derivatives(self, aircraft=None, dtheta_dot=0.005):
        """Determines the damping derivatives at the current state. Uses 
        a central difference sceme. Note, the damping derivatives are non-
        dimensionalized with respect to 2V/l_ref_lat and 2V/l_ref_lon.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the aerodynamic derivatives 
            of. Defaults to all aircraft in the scene.

        dtheta_dot : float
            The finite difference used to perturb the angular rates of the aircraft
            and determine the derivatives. Given in radians per second. Defaults to 0.005.

        Returns
        -------
        dict
            A dictionary of damping derivatives.
        """
        derivs = {}

        # Specify the aircraft
        if aircraft is None:
            aircraft_names = list(self._airplanes.keys())
        elif isinstance(aircraft, list):
            aircraft_names = copy.copy(aircraft)
        elif isinstance(aircraft, str):
            aircraft_names = [aircraft]
        else:
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft))

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}
            # Get current aerodynamic state
            _,_,vel_0 = self._airplanes[aircraft_name].get_aerodynamic_state()

            # Determine current angular rates
            omega_0 = self._airplanes[aircraft_name].w

            # Perturb forward in roll rate
            omega_pert_p_fwd = copy.copy(omega_0)
            omega_pert_p_fwd[0] += dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_p_fwd
            self.solve_forces(dimensional=False)
            FM_dp_fwd = self._FM

            # Perturb backward in roll rate
            omega_pert_p_bwd = copy.copy(omega_0)
            omega_pert_p_bwd[0] -= dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_p_bwd
            self.solve_forces(dimensional=False)
            FM_dp_bwd = self._FM

            # Perturb forward in pitch rate
            omega_pert_q_fwd = copy.copy(omega_0)
            omega_pert_q_fwd[1] += dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_q_fwd
            self.solve_forces(dimensional=False)
            FM_dq_fwd = self._FM

            # Perturb backward in pitch rate
            omega_pert_q_bwd = copy.copy(omega_0)
            omega_pert_q_bwd[1] -= dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_q_bwd
            self.solve_forces(dimensional=False)
            FM_dq_bwd = self._FM

            # Perturb forward in yaw rate
            omega_pert_r_fwd = copy.copy(omega_0)
            omega_pert_r_fwd[2] += dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_r_fwd
            self.solve_forces(dimensional=False)
            FM_dr_fwd = self._FM

            # Perturb backward in yaw rate
            omega_pert_r_bwd = copy.copy(omega_0)
            omega_pert_r_bwd[2] -= dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_r_bwd
            self.solve_forces(dimensional=False)
            FM_dr_bwd = self._FM

            # Reset state
            self._airplanes[aircraft_name].w = omega_0
            self._solved = False

            # Compute derivatives
            _, c, b = self.get_aircraft_reference_geometry(aircraft=aircraft_name)
            lat_non_dim = 2*vel_0/b
            lon_non_dim = 2*vel_0/c
            dx_inv = 1/(2*dtheta_dot)

            # With respect to roll rate
            derivs[aircraft_name]["CL,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CL"]-FM_dp_bwd[aircraft_name]["total"]["CL"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["CD,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CD"]-FM_dp_bwd[aircraft_name]["total"]["CD"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["CS,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CS"]-FM_dp_bwd[aircraft_name]["total"]["CS"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cx,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cx"]-FM_dp_bwd[aircraft_name]["total"]["Cx"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cy,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cy"]-FM_dp_bwd[aircraft_name]["total"]["Cy"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cz,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cz"]-FM_dp_bwd[aircraft_name]["total"]["Cz"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cl,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cl"]-FM_dp_bwd[aircraft_name]["total"]["Cl"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cm,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cm"]-FM_dp_bwd[aircraft_name]["total"]["Cm"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cn,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cn"]-FM_dp_bwd[aircraft_name]["total"]["Cn"])*dx_inv*lat_non_dim

            # With respect to pitch rate*dx_inv
            derivs[aircraft_name]["CL,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CL"]-FM_dq_bwd[aircraft_name]["total"]["CL"])*dx_inv*lon_non_dim
            derivs[aircraft_name]["CD,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CD"]-FM_dq_bwd[aircraft_name]["total"]["CD"])*dx_inv*lon_non_dim
            derivs[aircraft_name]["CS,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CS"]-FM_dq_bwd[aircraft_name]["total"]["CS"])*dx_inv*lon_non_dim
            derivs[aircraft_name]["Cx,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cx"]-FM_dq_bwd[aircraft_name]["total"]["Cx"])*dx_inv*lon_non_dim
            derivs[aircraft_name]["Cy,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cy"]-FM_dq_bwd[aircraft_name]["total"]["Cy"])*dx_inv*lon_non_dim
            derivs[aircraft_name]["Cz,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cz"]-FM_dq_bwd[aircraft_name]["total"]["Cz"])*dx_inv*lon_non_dim
            derivs[aircraft_name]["Cl,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cl"]-FM_dq_bwd[aircraft_name]["total"]["Cl"])*dx_inv*lon_non_dim
            derivs[aircraft_name]["Cm,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cm"]-FM_dq_bwd[aircraft_name]["total"]["Cm"])*dx_inv*lon_non_dim
            derivs[aircraft_name]["Cn,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cn"]-FM_dq_bwd[aircraft_name]["total"]["Cn"])*dx_inv*lon_non_dim

            # With respect to yaw rate*dx_inv
            derivs[aircraft_name]["CL,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CL"]-FM_dr_bwd[aircraft_name]["total"]["CL"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["CD,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CD"]-FM_dr_bwd[aircraft_name]["total"]["CD"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["CS,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CS"]-FM_dr_bwd[aircraft_name]["total"]["CS"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cx,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cx"]-FM_dr_bwd[aircraft_name]["total"]["Cx"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cy,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cy"]-FM_dr_bwd[aircraft_name]["total"]["Cy"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cz,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cz"]-FM_dr_bwd[aircraft_name]["total"]["Cz"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cl,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cl"]-FM_dr_bwd[aircraft_name]["total"]["Cl"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cm,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cm"]-FM_dr_bwd[aircraft_name]["total"]["Cm"])*dx_inv*lat_non_dim
            derivs[aircraft_name]["Cn,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cn"]-FM_dr_bwd[aircraft_name]["total"]["Cn"])*dx_inv*lat_non_dim

        return derivs


    def aircraft_control_derivatives(self, aircraft=None, dtheta=0.5):
        """Determines the control derivatives at the current state. Uses 
        a central difference sceme.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the aerodynamic derivatives 
            of. Defaults to all aircraft in the scene.

        dtheta : float
            The finite difference used to perturb the controls in degrees
            and determine the derivatives. Defaults to 0.5

        Returns
        -------
        dict
            A dictionary of control derivatives with respect to deflection in 
            radians.
        """
        derivs = {}

        # Specify the aircraft
        if aircraft is None:
            aircraft_names = self._airplanes.keys()
        elif isinstance(aircraft, list):
            aircraft_names = copy.copy(aircraft)
        elif isinstance(aircraft, str):
            aircraft_names = [aircraft]
        else:
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft))

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
                FM_fwd = self.solve_forces(dimensional=False)

                #Perturb forward
                pert_control_state[control_name] = curr_control_val - dtheta
                aircraft_object.set_control_state(control_state=pert_control_state)
                FM_bwd = self.solve_forces(dimensional=False)

                # Reset state
                pert_control_state[control_name] = curr_control_val
                aircraft_object.set_control_state(control_state=pert_control_state)
                self._solved = False

                # Calculate derivatives
                diff = 2*np.radians(dtheta)

                derivs[aircraft_name]["CL,d"+control_name] = (FM_fwd[aircraft_name]["total"]["CL"]-FM_bwd[aircraft_name]["total"]["CL"])/diff
                derivs[aircraft_name]["CD,d"+control_name] = (FM_fwd[aircraft_name]["total"]["CD"]-FM_bwd[aircraft_name]["total"]["CD"])/diff
                derivs[aircraft_name]["CS,d"+control_name] = (FM_fwd[aircraft_name]["total"]["CS"]-FM_bwd[aircraft_name]["total"]["CS"])/diff
                derivs[aircraft_name]["Cx,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cx"]-FM_bwd[aircraft_name]["total"]["Cx"])/diff
                derivs[aircraft_name]["Cy,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cy"]-FM_bwd[aircraft_name]["total"]["Cy"])/diff
                derivs[aircraft_name]["Cz,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cz"]-FM_bwd[aircraft_name]["total"]["Cz"])/diff
                derivs[aircraft_name]["Cl,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cl"]-FM_bwd[aircraft_name]["total"]["Cl"])/diff
                derivs[aircraft_name]["Cm,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cm"]-FM_bwd[aircraft_name]["total"]["Cm"])/diff
                derivs[aircraft_name]["Cn,d"+control_name] = (FM_fwd[aircraft_name]["total"]["Cn"]-FM_bwd[aircraft_name]["total"]["Cn"])/diff

        return derivs


    def aircraft_pitch_trim(self, **kwargs):
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
            raise IOError("aircraft_pitch_trim() may not be used when there is more than one aircraft in the scene.")
        try:
            self._constant_wind
        except:
            raise IOError("aircraft_pitch_trim() may not be used when the wind is not constant.")

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
            stab_derivs = self.aircraft_stability_derivatives()
            cont_derivs = self.aircraft_control_derivatives()
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
            self.set_aircraft_control_state({pitch_control : delta_flap1}, aircraft_name=aircraft_name)

        else: # Return to the original state
            airplane_object.set_aerodynamic_state(alpha=alpha_original)
            self.set_aircraft_control_state(controls_original, aircraft_name=aircraft_name)

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


    def aircraft_aero_center(self, **kwargs):
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
        aircraft at the current state. solve_forces() should be called before this function. 
        Angular distributions are given in radians.

        Parameters
        ----------
        filename : str
            Output file to write the distributions to. Saves as a .txt file. Defaults to no file.

        make_plots : list, optional
            List of keys from the dist dictionary to make plots of. A plot of the parameter as a function 
            of span fraction for each wing segment will then be generated and saved. This can create 
            a lot of plots!

        show_plots : bool, optional
            Whether to show the plots, rather than automatically saving them. Defaults to False.

        Returns
        -------
        dist : dict
            A dictionary containing lists of each parameter at each control point. The keys are the
            aircraft names. The nested keys are then "span_frac", "cpx", "cpy", "cpz", "chord", "twist", 
            "dihedral", "sweep", "area", "alpha", "Re", "M", "section_CL", "section_Cm", "section_parasitic_CD", 
            and "section_aL0".
        """

        # Make sure the LL equations have been solved in this state
        if not self._solved:
            self.solve_forces()

        # Make sure alpha has been calculated.
        v_i = np.sum(self._V_ji*self._gamma[:,np.newaxis,np.newaxis], axis=0)
        v_i += self._cp_v_inf
        v_ni = np.einsum('ij,ij->i', v_i, self._u_n)
        v_ai = np.einsum('ij,ij->i', v_i, self._u_a)
        self._alpha = np.arctan2(v_ni, v_ai)

        dist = {}

        index = 0

        # Setup table for saving to .txt file
        filename = kwargs.get("filename", None)
        if filename is not None:
            item_types = [("aircraft", "U18"),
                          ("segment", "U18"),
                          ("span_frac", "float"),
                          ("cpx", "float"),
                          ("cpy", "float"),
                          ("cpz", "float"),
                          ("chord", "float"),
                          ("twist", "float"),
                          ("dihedral", "float"),
                          ("sweep", "float"),
                          ("area", "float"),
                          ("alpha", "float"),
                          ("Re", "float"),
                          ("M", "float"),
                          ("section_CL", "float"),
                          ("section_Cm", "float"),
                          ("section_parasitic_CD", "float"),
                          ("section_aL0","float")]

            table_data = np.zeros(self._N, dtype=item_types)


        # Loop through airplanes
        for airplane_name in self._airplane_names:
            airplane_object = self._airplanes[airplane_name]
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
                dist[airplane_name][segment_name]["chord"] = list(self._c_bar[cur_slice])
                dist[airplane_name][segment_name]["twist"] = list(segment_object.twist_cp)
                dist[airplane_name][segment_name]["dihedral"] = list(segment_object.dihedral_cp)
                dist[airplane_name][segment_name]["sweep"] = list(segment_object.sweep_cp)
                dist[airplane_name][segment_name]["area"] = list(self._dS[cur_slice])

                # Airfoil info
                dist[airplane_name][segment_name]["section_CL"] = list(self._dL[cur_slice]/(self._q_i[cur_slice]*self._dS[cur_slice]))
                dist[airplane_name][segment_name]["section_Cm"] = list(self._Cm[cur_slice])
                dist[airplane_name][segment_name]["section_parasitic_CD"] = list(self._CD[cur_slice])
                dist[airplane_name][segment_name]["section_aL0"] = list(self._aL0[cur_slice])
                dist[airplane_name][segment_name]["alpha"] = list(np.degrees(self._alpha[cur_slice]))
                dist[airplane_name][segment_name]["Re"] = list(self._Re[cur_slice])
                dist[airplane_name][segment_name]["M"] = list(self._M[cur_slice])

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
                    table_data[cur_slice]["twist"] = dist[airplane_name][segment_name]["twist"]
                    table_data[cur_slice]["dihedral"] = dist[airplane_name][segment_name]["dihedral"]
                    table_data[cur_slice]["sweep"] = dist[airplane_name][segment_name]["sweep"]
                    table_data[cur_slice]["area"] = dist[airplane_name][segment_name]["area"]

                    # Airfoil info
                    table_data[cur_slice]["section_CL"] = dist[airplane_name][segment_name]["section_CL"]
                    table_data[cur_slice]["section_Cm"] = dist[airplane_name][segment_name]["section_Cm"]
                    table_data[cur_slice]["section_parasitic_CD"] = dist[airplane_name][segment_name]["section_parasitic_CD"]
                    table_data[cur_slice]["section_aL0"] = dist[airplane_name][segment_name]["section_aL0"]
                    table_data[cur_slice]["alpha"] = dist[airplane_name][segment_name]["alpha"]
                    table_data[cur_slice]["Re"] = dist[airplane_name][segment_name]["Re"]
                    table_data[cur_slice]["M"] = dist[airplane_name][segment_name]["M"]

                index += num_cps

        # Save distributions to .txt file
        if filename is not None:
            
            # Define header and output format
            header = "{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}{:<21}".format(
                "Aircraft", "Segment", "Span Fraction", "Control (x)", "Control (y)", "Control (z)", "Chord", "Twist", "Dihedral", "Sweep", "Area", "Alpha",
                "Re", "M", "CL", "Cm", "Parasitic CD", "Zero-Lift Alpha")
            format_string = "%-20s %-20s %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e %20.12e"

            # Save
            np.savetxt(filename, table_data, fmt=format_string, header=header)

        # Create plots specified by the user
        make_plots = kwargs.get("make_plots", [])
        for param in make_plots:
            for aircraft_name in self._airplane_names:
                for segment_name, segment_dist in dist[aircraft_name].items():
                    plt.figure()
                    plt.plot(segment_dist["span_frac"], segment_dist[param])
                    plt.xlabel("Span Fraction")
                    plt.ylabel(param)
                    plt.title(segment_name)
                    if kwargs.get("show_plots", False):
                        plt.show()
                    else:
                        plt.savefig("{0}_{1}_{2}_vs_span_fraction".format(aircraft_name, segment_name, param))
                    plt.close()

        return dist

    def get_aircraft_reference_geometry(self, aircraft=None):
        """Returns the reference geometries for the specified aircraft.

        Parameters
        ----------
        aircraft_name : str
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
            Number of points to use in dicretizing the airfoil section outlines. Defaults to 200.

        aircraft : str or list, optional
            Name(s) of the aircraft to include in the model. Defaults to all aircraft in the scene.
        """

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Model of single aircraft
        if len(aircraft_names) == 1:
            self._airplanes[aircraft_names[0]].export_stl(**kwargs)

        # Check for .stl file
        filename = kwargs.get("filename")
        if ".stl" not in filename:
            raise IOError("{0} is not a .stl file.".format(filename))

        # Multiple aircraft
        else:
            section_resolution = kwargs.get("section_resolution", 200)
            num_facets = 0
            vector_dict = {}

            # Loop through aircraft
            for aircraft_name in aircraft_names:
                airplane_object = self._airplanes[aircraft_name]
                vector_dict[aircraft_name] = {}

                # Loop through segments
                for segment_name, segment_object in airplane_object.wing_segments.items():
                    vectors = segment_object.get_stl_vectors(section_res=section_resolution)
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


    def aircraft_mean_aerodynamic_chord(self, **kwargs):
        """Returns the mean aerodynamic chord (MAC) for the specified aircraft.

        Parameters
        ----------
        aircraft_name : str
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


    def export_aircraft_stp(self, **kwargs):
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
        """

        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Loop through aircraft
        for aircraft_name in aircraft_names:
            self._airplanes[aircraft_name].export_stp(**kwargs)


    def export_aircraft_dxf(self, **kwargs):
        """Creates a .dxf file representing each lifting surface of the specified aircraft.

        Parameters
        ----------
        aircraft : str
            The aircraft to export .dxf files of.

        file_tag : str, optional
            Optional tag to prepend to output filename default. The output files will be named "<AIRCRAFT_NAME>_<WING_NAME>.dxf".

        section_resolution : int, optional
            Number of points to use in discretizing the airfoil section outline. Defaults to 200.
        """
        
        # Specify the aircraft
        aircraft_names = self._get_aircraft(**kwargs)

        # Loop through aircraft
        for aircraft_name in aircraft_names:
            self._airplanes[aircraft_name].export_dxf(**kwargs)


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