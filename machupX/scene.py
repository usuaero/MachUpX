from .helpers import _check_filepath, _convert_units, _vectorized_convert_units, _import_value, _quaternion_transform, _quaternion_inverse_transform
from .airplane import Airplane
from .standard_atmosphere import StandardAtmosphere

import json
import numpy as np
import scipy.interpolate as sinterp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import copy

class Scene:
    """A class defining a scene containing one or more aircraft.

    Parameters
    ----------
    scene_input : string or dict
        Dictionary or path to the JSON object specifying the scene parameters. 
        For information on creating input files, see examples/How_To_Create_Input_Files.

    Raises
    ------
    IOError
        If input filepath or filename is invalid

    Methods
    -------
    add_aircraft()

    set_aircraft_state()

    set_aircraft_control_state()

    solve_forces()

    display_wireframe()

    aircraft_derivatives()

    aircraft_stability_derivatives()

    aircraft_damping_derivatives()

    aircraft_control_derivatives()
    
    distributions()

    get_aircraft_reference_geometry()
    """

    def __init__(self, scene_input):

        self._airplanes = {}
        self._airplane_names = []
        self._segment_names = []
        self._N = 0
        self._num_aircraft = 0


        self._load_params(scene_input)


    def _load_params(self, scene_input):
        # Loads JSON object and stores input parameters and aircraft

        # File
        if isinstance(scene_input, str):
            _check_filepath(scene_input,".json")
            with open(scene_input) as input_json_handle:
                self._input_dict = json.load(input_json_handle)

        # Dictionary
        elif isinstance(scene_input, dict):
            self._input_dict = copy.deepcopy(scene_input)

        else:
            raise IOError("Input to Scene class initializer must be a file path or Python dictionary.")


        # Store solver parameters
        solver_params = self._input_dict.get("solver", {})
        self._nonlinear_solver = solver_params.get("type", "linear") == "nonlinear"
        self._solver_convergence = solver_params.get("convergence", 1e-10)
        self._solver_relaxation = solver_params.get("relaxation", 0.9)
        self._max_solver_iterations = solver_params.get("max_iterations", 100)

        # Store unit system
        self._unit_sys = self._input_dict.get("units", "English")

        # Setup atmospheric property getter functions
        self._atmos = StandardAtmosphere(unit_sys=self._unit_sys)
        self._get_density = self._initialize_density_getter()
        self._get_wind = self._initialize_wind_getter()

        # Initialize aircraft geometries
        for i, airplane_name in enumerate(self._input_dict["scene"]["aircraft"]):
            airplane_file = self._input_dict["scene"]["aircraft"][airplane_name]["file"]
            state = self._input_dict["scene"]["aircraft"][airplane_name].get("state",{})
            control_state = self._input_dict["scene"]["aircraft"][airplane_name].get("control_state",{})

            self.add_aircraft(airplane_name, airplane_file, state=state, control_state=control_state)


    def _initialize_density_getter(self):

        # Load value from dictionary
        input_dict = self._input_dict["scene"].get("atmosphere", {})
        default_density = self._atmos.rho(0.0)
        rho = _import_value("rho", input_dict, self._unit_sys, default_density)

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
                return self._atmos.rho(-position[2])
            
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
                    return self._density_field_interpolator(position).item()

        # Improper specification
        else:
            raise IOError("Density improperly specified as {0}.".format(rho))

        return density_getter

    
    def _initialize_wind_getter(self):
        
        # Load value from dict
        input_dict = self._input_dict["scene"].get("atmosphere", {})
        default_wind = [0,0,0]
        V_wind = _import_value("V_wind", input_dict, self._unit_sys, default_wind)

        if isinstance(V_wind, np.ndarray):

            if V_wind.shape == (3,): # Constant wind vector
                self._constant_wind = np.asarray(V_wind)

                def wind_getter(position):
                    return self._constant_wind

            else: # Array
                self._wind_data = V_wind
                
                # Create getters
                if self._wind_data.shape[1] is 6: # Wind field
                    self._wind_field_x_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3],self._wind_data[:,3])
                    self._wind_field_y_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3],self._wind_data[:,4])
                    self._wind_field_z_interpolator = sinterp.LinearNDInterpolator(self._wind_data[:,:3],self._wind_data[:,5])

                    def wind_getter(position):
                        Vx = self._wind_field_x_interpolator(position)
                        Vy = self._wind_field_y_interpolator(position)
                        Vz = self._wind_field_z_interpolator(position)
                        return np.asarray([Vx, Vy, Vz])

                elif self._wind_data.shape[1] is 4: # wind profile

                    def wind_getter(position):
                        Vx =  np.interp(-position[2], self._wind_data[:,0], self._wind_data[:,1])
                        Vy =  np.interp(-position[2], self._wind_data[:,0], self._wind_data[:,2])
                        Vz =  np.interp(-position[2], self._wind_data[:,0], self._wind_data[:,3])
                        return np.asarray([Vx, Vy, Vz])

                else:
                    raise IOError("Wind array has the wrong number of columns.")
        
        else:
            raise IOError("Wind velocity improperly specified as {0}".format(V_wind))

        return wind_getter


    def _get_wind_at_aircraft_origin(self, aircraft_name=None):
        # Returns the wind vector in flat-earth coordinates at the origin of the specified aircraft.
        # If there is only one aircraft in the scene, the aircraft does not need to be specified.

        # Specify the only aircraft if not already specified
        if aircraft_name is None:
            if self._num_aircraft == 1:
                aircraft_name = list(self._airplanes.keys())[0]
            else:
                raise IOError("Aircraft name must be specified if there is more than one aircraft in the scene.")

        aircraft_position = self._airplanes[aircraft_name].p_bar
        return self._get_wind(aircraft_position)

    
    def add_aircraft(self, airplane_name, airplane_input, state={}, control_state={}):
        """Inserts an aircraft into the scene. Note if an aircraft was already specified
        in the input file, it has already been added to the scene.

        Parameters
        ----------
        airplane_name : str
            Name of the airplane to be added.

        airplane_input : str or dict
            Path to the JSON object or dictionary describing the airplane.

        state : dict
            Dictionary describing the state of the airplane.

        control_state : dict
            Dictionary describing the state of the controls.

        Raises
        ------
        IOError
            If the input is invalid.

        """
        # Determine the local wind vector for setting the state of the aircraft
        aircraft_position = state.get("position", [0,0,0])
        v_wind = self._get_wind(aircraft_position)

        # Create and store the aircraft object
        self._airplanes[airplane_name] = Airplane(airplane_name, airplane_input, self._unit_sys, init_state=state, init_control_state=control_state, v_wind=v_wind)

        # Update member variables
        self._N += self._airplanes[airplane_name].get_num_cps()
        self._perform_geometry_calculations()
        self._num_aircraft += 1


    def _perform_geometry_calculations(self):
        # Performs calculations necessary for solving NLL which are only dependent on geometry.
        # This speeds up repeated calls to _solve(). This method should be called any time the 
        # geometry is updated, an aircraft is added to the scene, or the state of an aircraft 
        # changes. Note that all calculations occur in the flat-earth frame to all for multiple
        # aircraft.

        # Geometry
        self._c_bar = np.zeros(self._N) # Average chord
        self._dS = np.zeros(self._N) # Differential planform area
        self._PC = np.zeros((self._N,3)) # Control point location
        self._P0 = np.zeros((self._N,3)) # Inbound vortex node location
        self._P1 = np.zeros((self._N,3)) # Outbound vortex node location
        self._u_a = np.zeros((self._N,3)) # Section unit vectors
        self._u_n = np.zeros((self._N,3))
        self._u_s = np.zeros((self._N,3))

        index = 0
        self._airplane_names = []
        self._segment_names = []

        # Loop through airplanes
        for i, (airplane_name, airplane_object) in enumerate(self._airplanes.items()):
            # Store airplane and segment names to make sure they are always accessed in the same order
            self._airplane_names.append(airplane_name)
            self._segment_names.append([])

            # Loop through segments
            for segment_name, segment_object in airplane_object.wing_segments.items():
                self._segment_names[i].append(segment_name)
                num_cps = segment_object._N
                cur_slice = slice(index, index+num_cps)

                # Geometries
                self._PC[cur_slice,:] = airplane_object.p_bar+_quaternion_inverse_transform(airplane_object.q, segment_object.control_points)
                self._c_bar[cur_slice] = segment_object.c_bar_cp
                self._dS[cur_slice] = segment_object.dS

                node_points = segment_object.nodes
                self._P0[cur_slice,:] = airplane_object.p_bar+_quaternion_inverse_transform(airplane_object.q, node_points[:-1,:])
                self._P1[cur_slice,:] = airplane_object.p_bar+_quaternion_inverse_transform(airplane_object.q, node_points[1:,:])

                self._u_a[cur_slice,:] = _quaternion_inverse_transform(airplane_object.q, segment_object.u_a_cp)
                self._u_n[cur_slice,:] = _quaternion_inverse_transform(airplane_object.q, segment_object.u_n_cp)
                self._u_s[cur_slice,:] = _quaternion_inverse_transform(airplane_object.q, segment_object.u_s_cp)

                index += num_cps

        # Differential length vectors
        self._dl = self._P1 - self._P0

        # Spatial node vectors
        self._rj0i = self._PC-self._P0[:,np.newaxis]
        self._rj1i = self._PC-self._P1[:,np.newaxis]
        self._rj0i_mag = np.sqrt(np.einsum('ijk,ijk->ij', self._rj0i, self._rj0i))
        self._rj1i_mag = np.sqrt(np.einsum('ijk,ijk->ij', self._rj1i, self._rj1i))
        self._rj0i_rj1i_mag = self._rj0i_mag*self._rj1i_mag

        # Influence of bound vortex segment
        with np.errstate(divide='ignore', invalid='ignore'):
            numer = ((self._rj0i_mag+self._rj1i_mag)[:,:,np.newaxis]*np.cross(self._rj0i, self._rj1i))
            denom = self._rj0i_rj1i_mag*(self._rj0i_rj1i_mag+np.einsum('ijk,ijk->ij', self._rj0i, self._rj1i))
            self._V_ji_due_to_bound = np.true_divide(numer, denom[:,:,np.newaxis])
            diag_ind = np.diag_indices(self._N)
            self._V_ji_due_to_bound[diag_ind] = 0.0 # Ensure this actually comes out to be zero

        # Track whether the current scene state has been solved
        self._solved = False


    def _solve_linear(self, verbose=False):
        # Determines the vortex strengths of all horseshoe vortices in the scene using the linearize equations

        if verbose: print("Running linear solver...")
        start_time = time.time()

        # Gather necessary variables
        # Atmosphere
        self._rho = np.zeros(self._N)
        self._nu = np.zeros(self._N)
        self._a = np.ones(self._N)

        # Airfoil parameters
        alpha_approx = np.zeros(self._N)
        self._Re = np.zeros(self._N)
        self._M = np.zeros(self._N)
        CLa = np.zeros(self._N)
        self._aL0 = np.zeros(self._N)
        esp_f_delta_f = np.zeros(self._N)
        alpha_approx = np.zeros(self._N)

        # Velocities at vortex nodes
        P0_v_inf = np.zeros((self._N,3))
        P1_v_inf = np.zeros((self._N,3))

        # Velocities at control points
        self._cp_v_inf = np.zeros((self._N,3))
        cp_V_inf = np.zeros(self._N)
        cp_u_inf = np.zeros((self._N,3))

        # Aircraft velocities
        self._v_trans = np.zeros((self._num_aircraft,3))

        index = 0

        # Loop through airplanes
        for i, airplane_name in enumerate(self._airplane_names):
            airplane_object = self._airplanes[airplane_name]

            # Determine freestream velocity due to airplane translation
            self._v_trans[i,:] = -airplane_object.v

            # Loop through segments
            for segment_name in self._segment_names[i]:
                segment_object = airplane_object.wing_segments[segment_name]
                num_cps = segment_object._N
                cur_slice = slice(index, index+num_cps)

                # Freestream velocity at control points
                # Due to wind
                cp_v_wind = self._get_wind(self._PC[cur_slice,:])

                # Due to aircraft rotation (about the aircraft's center of gravity)
                cp_v_rot = _quaternion_inverse_transform(airplane_object.q, -np.cross(airplane_object.w, segment_object.control_points-airplane_object.CG))

                self._cp_v_inf[cur_slice,:] = self._v_trans[i,:]+cp_v_wind+cp_v_rot
                cp_V_inf[cur_slice] = np.linalg.norm(self._cp_v_inf[cur_slice,:], axis=1)
                cp_u_inf[cur_slice,:] = self._cp_v_inf[cur_slice]/cp_V_inf[cur_slice,np.newaxis]

                # Atmospheric density, speed of sound, and viscosity
                self._rho[cur_slice] = self._get_density(self._PC[cur_slice,:])
                self._a[cur_slice] = self._atmos.a(-self._PC[cur_slice,2])
                self._nu[cur_slice] = self._atmos.nu(-self._PC[cur_slice,2])

                # Freestream velocity at vortex nodes
                # Due to wind
                body_node_locs = segment_object.nodes
                global_node_locs = airplane_object.p_bar + _quaternion_inverse_transform(airplane_object.q, body_node_locs)
                node_v_wind = self._get_wind(global_node_locs)

                # Due to aircraft rotation (about the aircraft's center of gravity)
                node_v_rot = _quaternion_inverse_transform(airplane_object.q, -np.cross(airplane_object.w, body_node_locs-airplane_object.CG))

                node_v_inf = self._v_trans[i,:]+node_v_wind+node_v_rot
                P0_v_inf[cur_slice,:] = node_v_inf[:-1,:]
                P1_v_inf[cur_slice,:] = node_v_inf[1:,:]

                # Airfoil parameters
                alpha_approx[cur_slice] = np.einsum('ij,ij->i', cp_u_inf[cur_slice,:], self._u_n[cur_slice,:])
                self._Re[cur_slice] = cp_V_inf[cur_slice]*self._c_bar[cur_slice]/self._nu[cur_slice]
                self._M[cur_slice] = cp_V_inf[cur_slice]/self._a[cur_slice]

                airfoil_params = np.concatenate((alpha_approx[cur_slice,np.newaxis], self._Re[cur_slice,np.newaxis], self._M[cur_slice,np.newaxis]), axis=1)

                CLa[cur_slice] = segment_object.get_cp_CLa(airfoil_params)
                self._aL0[cur_slice] = segment_object.get_cp_aL0(airfoil_params)
                esp_f_delta_f[cur_slice] = segment_object.get_cp_flap()

                index += num_cps


        # Vortex node velocities
        P0_V_inf = np.linalg.norm(P0_v_inf, axis=1)
        P0_u_inf = P0_v_inf/P0_V_inf[:,np.newaxis]

        P1_V_inf = np.linalg.norm(P1_v_inf, axis=1)
        P1_u_inf = P1_v_inf/P1_V_inf[:,np.newaxis]

        # Influence of vortex segment 0
        denom = (self._rj0i_mag*(self._rj0i_mag-np.einsum('ijk,ijk->ij', P0_u_inf[np.newaxis], self._rj0i)))
        V_ji_due_to_0 = -np.cross(P0_u_inf, self._rj0i)/denom[:,:,np.newaxis]

        # Influence of vortex segment 1
        denom = (self._rj1i_mag*(self._rj1i_mag-np.einsum('ijk,ijk->ij', P1_u_inf[np.newaxis], self._rj1i)))
        V_ji_due_to_1 = np.cross(P1_u_inf, self._rj1i)/denom[:,:,np.newaxis]

        self._V_ji = 1/(4*np.pi)*(V_ji_due_to_0 + self._V_ji_due_to_bound + V_ji_due_to_1)
        self._V_ji_trans = self._V_ji.transpose((1,0,2))

        # A matrix
        A = np.zeros((self._N,self._N))
        V_ji_dot_u_n = np.einsum('ijk,ijk->ij', self._V_ji_trans, self._u_n[:,np.newaxis])
        A[:,:] = -(CLa*self._dS)[:,np.newaxis]*V_ji_dot_u_n
        diag_ind = np.diag_indices(self._N)
        u_inf_x_dl = np.cross(cp_u_inf, self._dl)
        A[diag_ind] += 2*np.sqrt(np.einsum('ij,ij->i', u_inf_x_dl, u_inf_x_dl))

        # b vector
        b = cp_V_inf*CLa*self._dS*(alpha_approx-self._aL0+esp_f_delta_f)

        # Solve
        self._Gamma = np.linalg.solve(A, b)

        end_time = time.time()
        return end_time-start_time


    def _solve_nonlinear(self, verbose=False):
        # Nonlinear improvement to the vector of gammas already determined
        if verbose: 
            print("Running nonlinear solver...")
            print("    Relaxation: {0}".format(self._solver_relaxation))
            print("    Convergence: {0}".format(self._solver_convergence))
            print("{0:<20}{1:<20}".format("Iteration", "SRSSQ Error"))
        start_time = time.time()

        J = np.zeros((self._N, self._N))

        # Airfoil coefs
        C_L = np.zeros(self._N)
        C_La = np.zeros(self._N)
        C_LRe = np.zeros(self._N)
        C_LM = np.zeros(self._N)

        # Airfoil params
        self._alpha = np.zeros(self._N)

        # Velocities
        v_i = np.zeros((self._N,3))
        v_ni = np.zeros(self._N)
        v_ai = np.zeros(self._N)

        iteration = 0
        error = 100
        while error > self._solver_convergence:
            iteration += 1

            # Calculate residual
            np.sum(self._V_ji*self._Gamma[:,np.newaxis,np.newaxis], axis=0, out=v_i)
            v_i += self._cp_v_inf
            np.einsum('ij,ij->i', v_i, self._u_n, out=v_ni)
            np.einsum('ij,ij->i', v_i, self._u_a, out=v_ai)
            V_i = np.sqrt(np.einsum('ij,ij->i', v_i, v_i))

            self._alpha = np.arctan2(v_ni, v_ai)
            self._Re = V_i*self._c_bar/self._nu
            self._M = V_i/self._a
            airfoil_params = np.concatenate((self._alpha[:,np.newaxis], self._Re[:,np.newaxis], self._M[:,np.newaxis]), axis=1)

            index = 0

            # Loop through airplanes
            for i, airplane_name in enumerate(self._airplane_names):
                airplane_object = self._airplanes[airplane_name]

                # Loop through segments
                for segment_name in self._segment_names[i]:
                    segment_object = airplane_object.wing_segments[segment_name]
                    num_cps = segment_object._N
                    cur_slice = slice(index, index+num_cps)

                    # Get lift coefficient
                    C_L[cur_slice] = segment_object.get_cp_CL(airfoil_params[cur_slice,:])
                    C_La[cur_slice] = segment_object.get_cp_CLa(airfoil_params[cur_slice,:])
                    C_LRe[cur_slice] = segment_object.get_cp_CLRe(airfoil_params[cur_slice,:])
                    C_LM[cur_slice] = segment_object.get_cp_CLM(airfoil_params[cur_slice,:])
                    self._aL0[cur_slice] = segment_object.get_cp_aL0(airfoil_params[cur_slice,:])

                    index += num_cps

            # Intermediate calcs
            w_i = np.cross(v_i, self._dl)
            w_i_mag = np.sqrt(np.einsum('ij,ij->i', w_i, w_i))
            v_iji = np.einsum('ijk,ijk->ij', v_i[:,np.newaxis,:], self._V_ji_trans)

            # Residual vector
            R = 2*w_i_mag*self._Gamma-V_i**2*C_L*self._dS

            error = np.sqrt(np.sum(R**2))

            # Caclulate Jacobian
            J[:,:] = (2*self._Gamma/w_i_mag)[:,np.newaxis]*(np.einsum('ijk,ijk->ij', w_i[:,np.newaxis,:], np.cross(self._V_ji_trans, self._dl)))
            J[:,:] -= (2*self._dS*C_L)[:,np.newaxis]*v_iji

            CL_gamma_alpha = C_La[:,np.newaxis]*(v_ai[:,np.newaxis]*np.einsum('ijk,ijk->ij', self._V_ji_trans, self._u_n[:,np.newaxis])-v_ni[:,np.newaxis]*np.einsum('ijk,ijk->ij', self._V_ji_trans, self._u_a[:,np.newaxis]))/(v_ni**2+v_ai**2)
            CL_gamma_Re = C_LRe*self._c_bar/(self._nu*V_i)[:,np.newaxis]*v_iji
            CL_gamma_M = C_LM/(self._a*V_i)[:,np.newaxis]*v_iji

            J[:,:] -= (V_i**2*self._dS)[:,np.newaxis]*(CL_gamma_alpha+CL_gamma_Re+CL_gamma_M)

            diag_ind = np.diag_indices(self._N)
            J[diag_ind] += 2*w_i_mag

            # Solve for change in gamma
            dGamma = np.linalg.solve(J, -R)

            # Update gammas
            self._Gamma += self._solver_relaxation*dGamma

            if verbose: print("{0:<20}{1:<20}".format(iteration, error))

            if iteration >= self._max_solver_iterations:
                print("Nonlinear solver failed to converge within the allowed number of iterations.")
                break
        else:
            if verbose: print("Nonlinear solver successfully converged.")

        end_time = time.time()
        return end_time-start_time


    def _integrate_forces_and_moments(self, non_dimensional=False, verbose=False):
        # Determines the forces and moments on each lifting surface
        start_time = time.time()

        # Calculate force differential elements
        induced_vels = self._Gamma[:,np.newaxis,np.newaxis]*self._V_ji
        v = self._cp_v_inf+np.sum(induced_vels, axis=0)
        dF_inv = (self._rho*self._Gamma)[:,np.newaxis]*np.cross(v, self._dl)
        self._CL = np.linalg.norm(dF_inv, axis=1)*np.sign(self._Gamma)

        # Calculate conditions for determining viscid contributions
        V = np.sqrt(np.einsum('ij,ij->i', v, v))
        u = v/V[:,np.newaxis]
        alpha = np.arctan2(np.einsum('ij,ij->i', v, self._u_n), np.einsum('ij,ij->i', v, self._u_a))
        airfoil_params = np.concatenate((alpha[:,np.newaxis], self._Re[:,np.newaxis], self._M[:,np.newaxis]), axis=1)
        q_inf = 0.5*self._rho*V**2

        # Store lift, drag, and moment coefficient distributions
        self._CD = np.zeros(self._N)
        self._Cm = np.zeros(self._N)

        r_CG = np.zeros((self._N,3))

        index = 0

        self._FM = {}

        # Loop through airplanes
        for i, airplane_name in enumerate(self._airplane_names):
            airplane_object = self._airplanes[airplane_name]
            FM_inv_airplane_total = np.zeros(9)
            FM_vis_airplane_total = np.zeros(9)
            if non_dimensional:
                self._FM[airplane_name] = {
                    "inviscid" : {
                        "CL" : {},
                        "CD" : {},
                        "CS" : {},
                        "Cx" : {},
                        "Cy" : {},
                        "Cz" : {},
                        "Cl" : {},
                        "Cm" : {},
                        "Cn" : {}
                    },
                    "viscous" : {
                        "CL" : {},
                        "CD" : {},
                        "CS" : {},
                        "Cx" : {},
                        "Cy" : {},
                        "Cz" : {},
                        "Cl" : {},
                        "Cm" : {},
                        "Cn" : {}
                    },
                    "total" : {}
                }
            else:
                self._FM[airplane_name] = {
                    "inviscid" : {
                        "FL" : {},
                        "FD" : {},
                        "FS" : {},
                        "Fx" : {},
                        "Fy" : {},
                        "Fz" : {},
                        "Mx" : {},
                        "My" : {},
                        "Mz" : {}
                    },
                    "viscous" : {
                        "FL" : {},
                        "FD" : {},
                        "FS" : {},
                        "Fx" : {},
                        "Fy" : {},
                        "Fz" : {},
                        "Mx" : {},
                        "My" : {},
                        "Mz" : {}
                    },
                    "total" : {}
                }

            # Determine freestream vector in body-fixed frame
            v_inf = self._v_trans[i,:] + self._get_wind(airplane_object.p_bar)
            V_inf = np.linalg.norm(v_inf)
            u_inf = _quaternion_transform(airplane_object.q, (v_inf/V_inf).flatten())

            # Determine reference parameters
            if non_dimensional:
                S_w =  airplane_object.S_w
                l_ref_lon = airplane_object.l_ref_lon
                l_ref_lat = airplane_object.l_ref_lat
                rho_ref = self._get_density(airplane_object.p_bar)
                q_ref = 0.5*rho_ref*V_inf**2

            # Loop through segments
            for segment_name in self._segment_names[i]:
                num_cps = airplane_object.wing_segments[segment_name]._N
                cur_slice = slice(index, index+num_cps)

                # Radii from control points to the aircraft's center of gravity
                r_CG[cur_slice,:] = self._PC[cur_slice,:]-(airplane_object.p_bar+_quaternion_inverse_transform(airplane_object.q, airplane_object.CG))[np.newaxis,:]

                # Determine viscid force
                # Get drag coef and redimensionalize
                self._CD[cur_slice] = self._airplanes[airplane_name].wing_segments[segment_name].get_cp_CD(airfoil_params[cur_slice,:])
                dD = q_inf[cur_slice]*self._dS[cur_slice]*self._CD[cur_slice]

                # Determine vector and translate back into body-fixed
                dF_b_visc = dD[:,np.newaxis]*u[cur_slice]
                F_b_visc = _quaternion_transform(airplane_object.q, np.sum(dF_b_visc, axis=0))
                L_visc, D_visc, S_visc = self._rotate_aero_forces(F_b_visc, u_inf)

                # Store
                if non_dimensional:
                    self._FM[airplane_name]["viscous"]["Cx"][segment_name] = F_b_visc[0].item()/(q_ref*S_w)
                    self._FM[airplane_name]["viscous"]["Cy"][segment_name] = F_b_visc[1].item()/(q_ref*S_w)
                    self._FM[airplane_name]["viscous"]["Cz"][segment_name] = F_b_visc[2].item()/(q_ref*S_w)

                    self._FM[airplane_name]["viscous"]["CL"][segment_name] = L_visc/(q_ref*S_w)
                    self._FM[airplane_name]["viscous"]["CD"][segment_name] = D_visc/(q_ref*S_w)
                    self._FM[airplane_name]["viscous"]["CS"][segment_name] = S_visc/(q_ref*S_w)
                else:
                    self._FM[airplane_name]["viscous"]["Fx"][segment_name] = F_b_visc[0].item()
                    self._FM[airplane_name]["viscous"]["Fy"][segment_name] = F_b_visc[1].item()
                    self._FM[airplane_name]["viscous"]["Fz"][segment_name] = F_b_visc[2].item()

                    self._FM[airplane_name]["viscous"]["FL"][segment_name] = L_visc
                    self._FM[airplane_name]["viscous"]["FD"][segment_name] = D_visc
                    self._FM[airplane_name]["viscous"]["FS"][segment_name] = S_visc

                # Inviscid
                # Determine vector and translate back into body-fixed
                F_b_inv = _quaternion_transform(airplane_object.q, np.sum(dF_inv[cur_slice], axis=0))
                L_inv, D_inv, S_inv = self._rotate_aero_forces(F_b_inv, u_inf)

                # Store
                if non_dimensional:
                    self._FM[airplane_name]["inviscid"]["Cx"][segment_name] = F_b_inv[0].item()/(q_ref*S_w)
                    self._FM[airplane_name]["inviscid"]["Cy"][segment_name] = F_b_inv[1].item()/(q_ref*S_w)
                    self._FM[airplane_name]["inviscid"]["Cz"][segment_name] = F_b_inv[2].item()/(q_ref*S_w)

                    self._FM[airplane_name]["inviscid"]["CL"][segment_name] = L_inv/(q_ref*S_w)
                    self._FM[airplane_name]["inviscid"]["CD"][segment_name] = D_inv/(q_ref*S_w)
                    self._FM[airplane_name]["inviscid"]["CS"][segment_name] = S_inv/(q_ref*S_w)
                else:
                    self._FM[airplane_name]["inviscid"]["Fx"][segment_name] = F_b_inv[0].item()
                    self._FM[airplane_name]["inviscid"]["Fy"][segment_name] = F_b_inv[1].item()
                    self._FM[airplane_name]["inviscid"]["Fz"][segment_name] = F_b_inv[2].item()

                    self._FM[airplane_name]["inviscid"]["FL"][segment_name] = L_inv
                    self._FM[airplane_name]["inviscid"]["FD"][segment_name] = D_inv
                    self._FM[airplane_name]["inviscid"]["FS"][segment_name] = S_inv

                # Determine viscid moment
                # Determine vector and rotate back to body-fixed
                dM_visc = dD[:,np.newaxis]*np.cross(r_CG[cur_slice], u[cur_slice])
                M_b_visc = _quaternion_transform(airplane_object.q, np.sum(dM_visc, axis=0))

                # Store
                if non_dimensional:
                    self._FM[airplane_name]["viscous"]["Cl"][segment_name] = M_b_visc[0].item()/(q_ref*S_w*l_ref_lat)
                    self._FM[airplane_name]["viscous"]["Cm"][segment_name] = M_b_visc[1].item()/(q_ref*S_w*l_ref_lon)
                    self._FM[airplane_name]["viscous"]["Cn"][segment_name] = M_b_visc[2].item()/(q_ref*S_w*l_ref_lat)
                else:
                    self._FM[airplane_name]["viscous"]["Mx"][segment_name] = M_b_visc[0].item()
                    self._FM[airplane_name]["viscous"]["My"][segment_name] = M_b_visc[1].item()
                    self._FM[airplane_name]["viscous"]["Mz"][segment_name] = M_b_visc[2].item()

                # Determine inviscid moment
                # Determine moment due to lift
                dM_vortex = np.cross(r_CG[cur_slice,:], dF_inv[cur_slice,:])

                # Determine moment due to section moment coef
                self._Cm[cur_slice] = self._airplanes[airplane_name].wing_segments[segment_name].get_cp_Cm(airfoil_params[cur_slice,:])
                dM_section = -(q_inf[cur_slice]*self._dS[cur_slice]*self._c_bar[cur_slice]*self._Cm[cur_slice])[:,np.newaxis]*self._u_s[cur_slice]

                # Rotate back to body-fixed
                M_b_inv = _quaternion_transform(airplane_object.q, np.sum(dM_section+dM_vortex, axis=0))

                #Store
                if non_dimensional:
                    self._FM[airplane_name]["inviscid"]["Cl"][segment_name] = M_b_inv[0].item()/(q_ref*S_w*l_ref_lat)
                    self._FM[airplane_name]["inviscid"]["Cm"][segment_name] = M_b_inv[1].item()/(q_ref*S_w*l_ref_lon)
                    self._FM[airplane_name]["inviscid"]["Cn"][segment_name] = M_b_inv[2].item()/(q_ref*S_w*l_ref_lat)
                else:
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

            else:
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
        # Takes the force vector and coverts it to lift, drag, and sideforce

        # Determine direction vectors
        u_lift = np.cross(u_inf,[0.,1.,0.])
        u_lift = u_lift/np.linalg.norm(u_lift)
        u_side = np.cross(u_lift,u_inf)
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


    def solve_forces(self, filename=None, non_dimensional=False, verbose=False):
        """Solves the NLL equations to determine the forces and moments on the aircraft.

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

        verbose : bool
            Display the time it took to complete each portion of the calculation. 
            Defaults to False.

        Returns
        -------
        dict:
            Dictionary of forces and moments acting on each wing segment.
        """
        linear_time = self._solve_linear(verbose=verbose)
        nonlinear_time = 0.0
        if self._nonlinear_solver:
            nonlinear_time = self._solve_nonlinear(verbose=verbose)
        integrate_time = self._integrate_forces_and_moments(non_dimensional=non_dimensional, verbose=verbose)

        if verbose:
            print("Time to compute linear solution: {0} s".format(linear_time))
            print("Time to compute nonlinear solution: {0} s".format(nonlinear_time))
            print("Time to integrate forces: {0} s".format(integrate_time))
            total_time = linear_time+nonlinear_time+integrate_time
            print("Total time: {0} s".format(total_time))
            print("Solution rate: {0} Hz".format(1/(total_time)))

        # Output to file
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
            How_To_Create_Input_Files. The state type may be 
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
        aircraft_position = state.get("position", [0,0,0])
        v_wind = self._get_wind(aircraft_position)

        # Set state and update precalcs for NLL
        self._airplanes[aircraft_name].set_state(state, v_wind=v_wind)
        self._perform_geometry_calculations()


    def set_aircraft_control_state(self, control_state={}, aircraft_name=None):
        """Sets the control state of the given aircraft.

        Parameters
        ----------
        control_state : dict
            Dictionary describing the control state:

            {
                "<CONTROL_NAME>" : deflection, float
                ...
            }

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


    def display_wireframe(self, show_legend=False, filename=None):
        """Displays a 3D wireframe plot of the scene.

        Parameters
        ----------
        show_legend : bool
            If this is set to True, a legend will appear detailing which color corresponds to which wing segment.
            Otherwise, the wing segments are all black.

        filename : str
            File to save an image of the wireframe to. If specified, the wireframe will not be 
            automatically displayed. If not specified, the wireframe will display to the user 
            and not save.
        """

        segment_names = []

        fig = plt.figure(figsize=plt.figaspect(1.0))
        ax = fig.gca(projection='3d')

        first_segment = True

        # Loop through airplanes
        for airplane_name, airplane_object in self._airplanes.items():

            # Loop through segments
            for segment_name, segment_object in airplane_object.wing_segments.items():
                segment_names.append(segment_name)

                points = airplane_object.p_bar+_quaternion_transform(airplane_object.q, segment_object.get_outline_points())
                if show_legend: # Then colors matter
                    ax.plot(points[:,0], points[:,1], points[:,2], '-')
                else:
                    ax.plot(points[:,0], points[:,1], points[:,2], 'k-')

                if first_segment:
                    x_lims = [min(points[:,0].flatten()), max(points[:,0].flatten())]
                    y_lims = [min(points[:,1].flatten()), max(points[:,1].flatten())]
                    z_lims = [min(points[:,2].flatten()), max(points[:,2].flatten())]
                    first_segment = False
                else:
                    x_lims = [min(x_lims[0], min(points[:,0].flatten())), max(x_lims[1], max(points[:,0].flatten()))]
                    y_lims = [min(y_lims[0], min(points[:,1].flatten())), max(y_lims[1], max(points[:,1].flatten()))]
                    z_lims = [min(z_lims[0], min(points[:,2].flatten())), max(z_lims[1], max(points[:,2].flatten()))]


        if show_legend:
            ax.legend(segment_names)

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        # Set axes to the same scale
        x_diff = x_lims[1]-x_lims[0]
        y_diff = y_lims[1]-y_lims[0]
        z_diff = z_lims[1]-z_lims[0]
        max_diff = max([x_diff, y_diff, z_diff])

        x_cent = x_lims[0]+0.5*x_diff
        y_cent = y_lims[0]+0.5*y_diff
        z_cent = z_lims[0]+0.5*z_diff

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

        if filename is not None:
            plt.savefig(filename)
        else:
            plt.show()
        plt.close()


    def aircraft_derivatives(self, aircraft=None, filename=None):
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
            # Determine stability derivatives
            derivs[aircraft_name]["stability"] = self.aircraft_stability_derivatives(aircraft=aircraft_name)[aircraft_name]
        
            # Determine damping derivatives
            derivs[aircraft_name]["damping"] = self.aircraft_damping_derivatives(aircraft=aircraft_name)[aircraft_name]

            # Determine control derivatives
            derivs[aircraft_name]["control"] = self.aircraft_control_derivatives(aircraft=aircraft_name)[aircraft_name]

        # Export to file
        if filename is not None:
            with open(filename, 'w') as output_handle:
                json.dump(derivs, output_handle)

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
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft_name))

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}
            # Get current aerodynamic state
            alpha_0, beta_0,_ = self._airplanes[aircraft_name].get_aerodynamic_state()

            # Perturb forward in alpha
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0+dtheta)
            self.solve_forces(non_dimensional=True)
            FM_dalpha_fwd = self._FM

            # Perturb backward in alpha
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0-dtheta)
            self.solve_forces(non_dimensional=True)
            FM_dalpha_bwd = self._FM

            # Perturb forward in beta
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0, beta=beta_0+dtheta) # We have to reset alpha on this one
            self.solve_forces(non_dimensional=True)
            FM_dbeta_fwd = self._FM

            # Perturb backward in beta
            self._airplanes[aircraft_name].set_aerodynamic_state(beta=beta_0-dtheta)
            self.solve_forces(non_dimensional=True)
            FM_dbeta_bwd = self._FM

            # Derivatives with respect to alpha
            diff = 2*np.radians(dtheta) # The derivative is in radians

            derivs[aircraft_name]["CL,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CL"]-FM_dalpha_bwd[aircraft_name]["total"]["CL"])/diff
            derivs[aircraft_name]["CD,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CD"]-FM_dalpha_bwd[aircraft_name]["total"]["CD"])/diff
            derivs[aircraft_name]["CS,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["CS"]-FM_dalpha_bwd[aircraft_name]["total"]["CS"])/diff
            derivs[aircraft_name]["Cx,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cx"]-FM_dalpha_bwd[aircraft_name]["total"]["Cx"])/diff
            derivs[aircraft_name]["Cy,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cy"]-FM_dalpha_bwd[aircraft_name]["total"]["Cy"])/diff
            derivs[aircraft_name]["Cz,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cz"]-FM_dalpha_bwd[aircraft_name]["total"]["Cz"])/diff
            derivs[aircraft_name]["Cl,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cl"]-FM_dalpha_bwd[aircraft_name]["total"]["Cl"])/diff
            derivs[aircraft_name]["Cm,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cm"]-FM_dalpha_bwd[aircraft_name]["total"]["Cm"])/diff
            derivs[aircraft_name]["Cn,a"] = (FM_dalpha_fwd[aircraft_name]["total"]["Cn"]-FM_dalpha_bwd[aircraft_name]["total"]["Cn"])/diff

            # Derivatives with respect to beta
            derivs[aircraft_name]["CL,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["CL"]-FM_dbeta_bwd[aircraft_name]["total"]["CL"])/diff
            derivs[aircraft_name]["CD,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["CD"]-FM_dbeta_bwd[aircraft_name]["total"]["CD"])/diff
            derivs[aircraft_name]["CS,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["CS"]-FM_dbeta_bwd[aircraft_name]["total"]["CS"])/diff
            derivs[aircraft_name]["Cx,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cx"]-FM_dbeta_bwd[aircraft_name]["total"]["Cx"])/diff
            derivs[aircraft_name]["Cy,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cy"]-FM_dbeta_bwd[aircraft_name]["total"]["Cy"])/diff
            derivs[aircraft_name]["Cz,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cz"]-FM_dbeta_bwd[aircraft_name]["total"]["Cz"])/diff
            derivs[aircraft_name]["Cl,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cl"]-FM_dbeta_bwd[aircraft_name]["total"]["Cl"])/diff
            derivs[aircraft_name]["Cm,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cm"]-FM_dbeta_bwd[aircraft_name]["total"]["Cm"])/diff
            derivs[aircraft_name]["Cn,B"] = (FM_dbeta_fwd[aircraft_name]["total"]["Cn"]-FM_dbeta_bwd[aircraft_name]["total"]["Cn"])/diff
        
            # Reset aerodynamic state
            self._airplanes[aircraft_name].set_aerodynamic_state(alpha=alpha_0, beta=beta_0)

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
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft_name))

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
            self.solve_forces(non_dimensional=True)
            FM_dp_fwd = self._FM

            # Perturb backward in roll rate
            omega_pert_p_bwd = copy.copy(omega_0)
            omega_pert_p_bwd[0] -= dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_p_bwd
            self.solve_forces(non_dimensional=True)
            FM_dp_bwd = self._FM

            # Perturb forward in pitch rate
            omega_pert_q_fwd = copy.copy(omega_0)
            omega_pert_q_fwd[1] += dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_q_fwd
            self.solve_forces(non_dimensional=True)
            FM_dq_fwd = self._FM

            # Perturb backward in pitch rate
            omega_pert_q_bwd = copy.copy(omega_0)
            omega_pert_q_bwd[1] -= dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_q_bwd
            self.solve_forces(non_dimensional=True)
            FM_dq_bwd = self._FM

            # Perturb forward in yaw rate
            omega_pert_r_fwd = copy.copy(omega_0)
            omega_pert_r_fwd[2] += dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_r_fwd
            self.solve_forces(non_dimensional=True)
            FM_dr_fwd = self._FM

            # Perturb backward in yaw rate
            omega_pert_r_bwd = copy.copy(omega_0)
            omega_pert_r_bwd[2] -= dtheta_dot
            self._airplanes[aircraft_name].w = omega_pert_r_bwd
            self.solve_forces(non_dimensional=True)
            FM_dr_bwd = self._FM

            # Reset state
            self._airplanes[aircraft_name].w = omega_0

            # Compute derivatives
            _, c, b = self.get_aircraft_reference_geometry(aircraft=aircraft_name)

            # With respect to roll rate
            derivs[aircraft_name]["CL,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CL"]-FM_dp_bwd[aircraft_name]["total"]["CL"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["CD,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CD"]-FM_dp_bwd[aircraft_name]["total"]["CD"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["CS,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["CS"]-FM_dp_bwd[aircraft_name]["total"]["CS"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cx,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cx"]-FM_dp_bwd[aircraft_name]["total"]["Cx"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cy,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cy"]-FM_dp_bwd[aircraft_name]["total"]["Cy"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cz,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cz"]-FM_dp_bwd[aircraft_name]["total"]["Cz"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cl,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cl"]-FM_dp_bwd[aircraft_name]["total"]["Cl"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cm,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cm"]-FM_dp_bwd[aircraft_name]["total"]["Cm"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cn,pbar"] = (FM_dp_fwd[aircraft_name]["total"]["Cn"]-FM_dp_bwd[aircraft_name]["total"]["Cn"])/(2*dtheta_dot)*2*vel_0/b

            # With respect to pitch rate
            derivs[aircraft_name]["CL,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CL"]-FM_dq_bwd[aircraft_name]["total"]["CL"])/(2*dtheta_dot)*2*vel_0/c
            derivs[aircraft_name]["CD,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CD"]-FM_dq_bwd[aircraft_name]["total"]["CD"])/(2*dtheta_dot)*2*vel_0/c
            derivs[aircraft_name]["CS,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["CS"]-FM_dq_bwd[aircraft_name]["total"]["CS"])/(2*dtheta_dot)*2*vel_0/c
            derivs[aircraft_name]["Cx,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cx"]-FM_dq_bwd[aircraft_name]["total"]["Cx"])/(2*dtheta_dot)*2*vel_0/c
            derivs[aircraft_name]["Cy,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cy"]-FM_dq_bwd[aircraft_name]["total"]["Cy"])/(2*dtheta_dot)*2*vel_0/c
            derivs[aircraft_name]["Cz,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cz"]-FM_dq_bwd[aircraft_name]["total"]["Cz"])/(2*dtheta_dot)*2*vel_0/c
            derivs[aircraft_name]["Cl,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cl"]-FM_dq_bwd[aircraft_name]["total"]["Cl"])/(2*dtheta_dot)*2*vel_0/c
            derivs[aircraft_name]["Cm,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cm"]-FM_dq_bwd[aircraft_name]["total"]["Cm"])/(2*dtheta_dot)*2*vel_0/c
            derivs[aircraft_name]["Cn,qbar"] = (FM_dq_fwd[aircraft_name]["total"]["Cn"]-FM_dq_bwd[aircraft_name]["total"]["Cn"])/(2*dtheta_dot)*2*vel_0/c

            # With respect to yaw rate
            derivs[aircraft_name]["CL,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CL"]-FM_dr_bwd[aircraft_name]["total"]["CL"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["CD,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CD"]-FM_dr_bwd[aircraft_name]["total"]["CD"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["CS,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["CS"]-FM_dr_bwd[aircraft_name]["total"]["CS"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cx,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cx"]-FM_dr_bwd[aircraft_name]["total"]["Cx"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cy,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cy"]-FM_dr_bwd[aircraft_name]["total"]["Cy"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cz,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cz"]-FM_dr_bwd[aircraft_name]["total"]["Cz"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cl,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cl"]-FM_dr_bwd[aircraft_name]["total"]["Cl"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cm,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cm"]-FM_dr_bwd[aircraft_name]["total"]["Cm"])/(2*dtheta_dot)*2*vel_0/b
            derivs[aircraft_name]["Cn,rbar"] = (FM_dr_fwd[aircraft_name]["total"]["Cn"]-FM_dr_bwd[aircraft_name]["total"]["Cn"])/(2*dtheta_dot)*2*vel_0/b

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
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft_name))

        for aircraft_name in aircraft_names:
            derivs[aircraft_name] = {}
            aircraft_object = self._airplanes[aircraft_name]
            curr_control_state = aircraft_object.current_control_state
            pert_control_state = copy.deepcopy(curr_control_state)

            # Loop through available controls
            for control_name in aircraft_object.control_names:

                curr_control_val = curr_control_state.get(control_name, 0.0)

                #Perturb forward
                pert_control_state[control_name] = curr_control_val + dtheta
                aircraft_object.set_control_state(control_state=pert_control_state)
                FM_fwd = self.solve_forces(non_dimensional=True)

                #Perturb forward
                pert_control_state[control_name] = curr_control_val - dtheta
                aircraft_object.set_control_state(control_state=pert_control_state)
                FM_bwd = self.solve_forces(non_dimensional=True)

                # Reset state
                pert_control_state[control_name] = curr_control_val

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


    def aircraft_pitch_trim(self, aircraft=None, iterations=1):
        """Returns the required angle of attack and elevator deflection for trim at the current state.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the aerodynamic derivatives 
            of. Defaults to all aircraft in the scene.

        iterations : int
            The number of times to loop through and trim each aircraft in the scene. Multiple iterations
            may be required for situations where each aircraft heavily influences the others. Defaults 
            to 1.

        Returns
        -------
        trim_angles : dict
            {
                "<AIRCRAFT_NAME>" : {
                    "alpha" : 0.0,
                    "delevator" : 0.0
                }
            }
        """

        # Specify the aircraft
        if aircraft is None:
            aircraft_names = self._airplanes.keys()
        elif isinstance(aircraft, list):
            aircraft_names = copy.copy(aircraft)
        elif isinstance(aircraft, str):
            aircraft_names = list(aircraft)
        else:
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft_name))

        for i in range(iterations):
            for aircraft_name in aircraft_names:
                # Store the current orientation
                q0 = copy.copy(self._airplanes[aircraft_name].q)


    def aircraft_aero_center(self, aircraft=None):
        """Returns the location of the aerodynamic center of the aircraft at the current state.

        Parameters
        ----------
        aircraft : str or list
            The name(s) of the aircraft to determine the aerodynamic derivatives 
            of. Defaults to all aircraft in the scene.

        Returns
        -------
        ac_loc : dict
            The location of the aerodynamic center in body-fixed coordinates for each aircraft.
            {
                "<AIRCRAFT_NAME>" : [0.0, 0.0, 0.0]
            }
        """

        # Specify the aircraft
        if aircraft is None:
            aircraft_names = self._airplanes.keys()
        elif isinstance(aircraft, list):
            aircraft_names = copy.copy(aircraft)
        elif isinstance(aircraft, str):
            aircraft_names = list(aircraft)
        else:
            raise IOError("{0} is not an allowable aircraft name specification.".format(aircraft_name))

        pass


    def distributions(self, filename=None, make_plots=[]):
        """Returns various parameters, as well as forces and moments, at each control point for all
        aircraft at the current state. solve_forces() should be called before this function. 
        Angular distributions are given in radians.

        Parameters
        ----------
        filename : str
            Output file to write the distributions to. Defaults to no file.

        make_plots : list
            List of keys from the dist dictionary to make plots of. A plot of the parameter as a function 
            of span fraction for each wing segment will then be generated and saved. This can create 
            a lot of plots!

        Returns
        -------
        dist : dict
            A dictionary containing lists of each parameter at each control point. The keys are the
            aircraft names. The nested keys are then "span_frac", cpx", "cpy", "cpz", "chord", "twist", 
            "dihedral", "sweep", "area", "alpha", "Re", "M", "section_CL", "section_Cm", "section_parasitic_CD", 
            and "section_aL0".
        """
        if not self._solved:
            self.solve_forces()

        dist = {}

        index = 0

        # Loop through airplanes
        for i, airplane_name in enumerate(self._airplane_names):
            airplane_object = self._airplanes[airplane_name]
            airplane_cps = airplane_object.get_num_cps()
            airplane_slice = slice(index, index+airplane_cps)
            dist[airplane_name] = {}

            # Loop through segments
            for segment_name in self._segment_names[i]:
                segment_object = airplane_object.wing_segments[segment_name]
                num_cps = segment_object._N
                cur_slice = slice(index, index+num_cps)
                dist[airplane_name][segment_name] = {}

                # Control point locations
                dist[airplane_name][segment_name]["span_frac"] = list(segment_object._cp_span_locs)
                dist[airplane_name][segment_name]["cpx"] = list(self._PC[cur_slice,0])
                dist[airplane_name][segment_name]["cpy"] = list(self._PC[cur_slice,1])
                dist[airplane_name][segment_name]["cpz"] = list(self._PC[cur_slice,2])

                # Geometry
                dist[airplane_name][segment_name]["chord"] = list(self._c_bar[cur_slice])
                dist[airplane_name][segment_name]["twist"] = segment_object.twist_cp
                dist[airplane_name][segment_name]["dihedral"] = segment_object.dihedral_cp
                dist[airplane_name][segment_name]["sweep"] = segment_object.sweep_cp
                dist[airplane_name][segment_name]["area"] = list(self._dS[cur_slice])

                # Airfoil info
                dist[airplane_name][segment_name]["section_CL"] = list(self._CL[cur_slice])
                dist[airplane_name][segment_name]["section_Cm"] = list(self._Cm[cur_slice])
                dist[airplane_name][segment_name]["section_parasitic_CD"] = list(self._CD[cur_slice])
                dist[airplane_name][segment_name]["section_aL0"] = list(self._aL0[cur_slice])
                dist[airplane_name][segment_name]["alpha"] = list(self._alpha[cur_slice])
                dist[airplane_name][segment_name]["Re"] = list(self._Re[cur_slice])
                dist[airplane_name][segment_name]["M"] = list(self._M[cur_slice])

                index += num_cps

        if filename is not None:
            with open(filename, 'w') as output_handle:
                json.dump(dist, filename)

        for param in make_plots:
            for segment_name, segment_dist in dist["test_plane"].items():
                plt.figure()
                plt.plot(segment_dist["span_frac"], segment_dist[param])
                plt.xlabel("Span Fraction")
                plt.ylabel(param)
                plt.title(segment_name)
                plt.savefig("{0}_{1}_vs_span_fraction".format(segment_name, param))
                plt.close()

        return dist

    def get_aircraft_reference_geometry(self, aircraft=None):
        """Returns the reference geometries for the specified aircraft.

        Parameters
        ----------
        aircraft_name : str
            The name of the aircraft to get the reference params for. Does
            not need to be specified if there is only one aircraft in the 
            scene.

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
                aircraft_name = list(self._airplanes.keys())[0]
            else:
                raise IOError("Aircraft name must be specified if there is more than one aircraft in the scene.")

        airplane_object = self._airplanes[aircraft]
        return airplane_object.S_w, airplane_object.l_ref_lon, airplane_object.l_ref_lat