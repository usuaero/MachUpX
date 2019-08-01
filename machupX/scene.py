from .helpers import _check_filepath, _convert_units, _vectorized_convert_units, _import_value, _quaternion_transform, _quaternion_inverse_transform
from .airplane import Airplane

import json
from skaero.atmosphere import coesa
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

    solve_forces()

    display_wireframe()
    """

    def __init__(self, scene_input):

        self.airplanes = {}
        self._airplane_names = []
        self._segment_names = []
        self._N = 0

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
        self._unit_sys = self._input_dict.get("units","English")

        # Initialize aircraft geometries
        for i, airplane_name in enumerate(self._input_dict["scene"]["aircraft"]):
            airplane_file = self._input_dict["scene"]["aircraft"][airplane_name]["file"]
            state = self._input_dict["scene"]["aircraft"][airplane_name].get("state",{})
            control_state = self._input_dict["scene"]["aircraft"][airplane_name].get("control_state",{})

            self.add_aircraft(airplane_name, airplane_file, state=state, control_state=control_state)

        # Setup atmospheric property getter functions
        self._get_density = self._initialize_density_getter()
        self._get_wind = self._initialize_wind_getter()


    def _initialize_density_getter(self):

        # Load value from dictionary
        input_dict = self._input_dict["scene"].get("atmosphere", {})
        default_density = self._density_from_atmos_table(0.0, "standard")
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
                return self._density_from_atmos_table(position[2], rho)
            
        # Array
        elif isinstance(rho, np.ndarray):
            self._density_data = rho

            # Create getters
            if self._density_data.shape[1] is 2: # Density profile

                def density_getter(position):
                    return np.interp(position[2], self._density_data[:,0], self._density_data[:,1])

            elif self._density_data.shape[1] is 4: # Density field
                self._density_field_interpolator = sinterp.LinearNDInterpolator(self._density_data[:,:3],self._density_data[:,3])

                def density_getter(position):
                    return self._density_field_interpolator(position).item()

        # Improper specification
        else:
            raise IOError("Density improperly specified as {0}.".format(rho))

        return density_getter
        

    def _density_from_atmos_table(self, altitude, profile):
        # Computes the density at a given altitude

        if self._unit_sys == "English":
            alt = _convert_units(altitude, "ft", "SI")
        else: 
            alt = altitude

        if profile == "standard":
            _,_,_,rho = coesa.table(alt)

        if self._unit_sys == "English":
            rho = _convert_units(rho, "kg/m^3", "English")

        return rho

    
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
                        Vx =  np.interp(position[2], self._wind_data[:,0], self._wind_data[:,1])
                        Vy =  np.interp(position[2], self._wind_data[:,0], self._wind_data[:,2])
                        Vz =  np.interp(position[2], self._wind_data[:,0], self._wind_data[:,3])
                        return np.asarray([Vx, Vy, Vz])

                else:
                    raise IOError("Wind array has the wrong number of columns.")
        
        else:
            raise IOError("Wind velocity improperly specified as {0}".format(V_wind))

        return wind_getter

    
    def add_aircraft(self, airplane_name, airplane_file, state={}, control_state={}):
        """Inserts an aircraft into the scene

        Parameters
        ----------
        airplane_name : str
            Name of the airplane to be added.

        ID : int
            ID of the airplane in the scene.
        
        airplane_file : str
            Path to the JSON object describing the airplane.

        state : dict
            Dictionary describing the state of the airplane.

        control_state : dict
            Dictionary describing the state of the controls.

        Returns
        -------

        Raises
        ------
        IOError
            If the input is invalid.

        """

        self.airplanes[airplane_name] = Airplane(airplane_name, airplane_file, self._unit_sys, state=state, control_state=control_state)
        self._N += self.airplanes[airplane_name].get_num_cps()
        self._perform_geometry_calculations()


    def _perform_geometry_calculations(self):
        # Performs calculations necessary for solving NLL which are only dependent on geometry.
        # This speeds up repeated calls to _solve(). This method should be called any time the 
        # geometry is updated.

        # Geometry
        self._c_bar = np.zeros(self._N)
        self._dS = np.zeros(self._N)
        self._PC = np.zeros((self._N,3))
        self._P0 = np.zeros((self._N,3))
        self._P1 = np.zeros((self._N,3))
        self._u_a = np.zeros((self._N,3))
        self._u_n = np.zeros((self._N,3))
        self._u_s = np.zeros((self._N,3))

        index = 0

        # Loop through airplanes
        for i, (airplane_name, airplane_object) in enumerate(self.airplanes.items()):
            # Store airplane and segment names to make sure they are always accessed in the same order
            self._airplane_names.append(airplane_name)
            self._segment_names.append([])

            # Loop through segments
            for segment_name, segment_object in airplane_object.wing_segments.items():
                self._segment_names[i].append(segment_name)
                num_cps = segment_object._N
                cur_slice = slice(index, index+num_cps)

                # Geometries
                body_cp_locs = segment_object.control_points
                self._PC[cur_slice,:] = body_cp_locs
                self._c_bar[cur_slice] = segment_object.c_bar_cp
                self._dS[cur_slice] = segment_object.dS

                node_points = segment_object.nodes
                self._P0[cur_slice,:] = node_points[:-1,:]
                self._P1[cur_slice,:] = node_points[1:,:]

                self._u_a[cur_slice,:] = segment_object.u_a_cp
                self._u_n[cur_slice,:] = segment_object.u_n_cp
                self._u_s[cur_slice,:] = segment_object.u_s_cp

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


    def _solve_linear(self, verbose=False):
        # Determines the vortex strengths of all horseshoe vortices in the scene using the linearize equations

        if verbose: print("Running linear solver...")
        start_time = time.time()

        # Gather necessary variables
        airplanes = []
        segments = []

        # Atmosphere
        self._rho = np.zeros(self._N)

        # Airfoil parameters
        alpha_approx = np.zeros(self._N)
        CLa = np.zeros(self._N)
        aL0 = np.zeros(self._N)

        # Velocities at vortex nodes
        P0_v_inf = np.zeros((self._N,3))
        P1_v_inf = np.zeros((self._N,3))

        # Velocities at control points
        self._cp_v_inf = np.zeros((self._N,3))

        index = 0

        # Loop through airplanes
        for i, airplane_name in enumerate(self._airplane_names):
            airplane_object = self.airplanes[airplane_name]

            # Determine freestream velocity due to airplane translation
            v_trans = airplane_object.get_v_inf()

            # Loop through segments
            for segment_name in self._segment_names[i]:
                segment_object = airplane_object.wing_segments[segment_name]
                num_cps = segment_object._N
                cur_slice = slice(index, index+num_cps)

                # Freestream velocity at control points
                # Due to wind
                global_cp_locs = airplane_object.p_bar + _quaternion_inverse_transform(airplane_object.q, self._PC[cur_slice,:])
                cp_v_wind = _quaternion_transform(airplane_object.q, self._get_wind(global_cp_locs))

                # Due to aircraft rotation
                cp_v_rot = -np.cross(airplane_object.w, self._PC[cur_slice,:])

                self._cp_v_inf[cur_slice,:] = v_trans+cp_v_wind+cp_v_rot

                # Atmospheric density
                self._rho[cur_slice] = self._get_density(global_cp_locs)

                # Freestream velocity at vortex nodes
                # Due to wind
                body_node_locs = segment_object.nodes
                global_node_locs = airplane_object.p_bar + _quaternion_inverse_transform(airplane_object.q, body_node_locs)
                node_v_wind = _quaternion_transform(airplane_object.q, self._get_wind(global_node_locs))

                # Due to aircraft rotation
                node_v_rot = -np.cross(airplane_object.w, body_node_locs)

                node_v_inf = v_trans+node_v_wind+node_v_rot
                P0_v_inf[cur_slice,:] = node_v_inf[:-1,:]
                P1_v_inf[cur_slice,:] = node_v_inf[1:,:]

                # Airfoil parameters
                CLa[cur_slice] = segment_object.get_cp_CLa()
                aL0[cur_slice] = segment_object.get_cp_aL0()

                index += num_cps

        # Control point velocities
        cp_V_inf = np.linalg.norm(self._cp_v_inf, axis=1)
        cp_u_inf = self._cp_v_inf/cp_V_inf[:,np.newaxis]

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
        b = cp_V_inf*CLa*self._dS*(np.einsum('ij,ij->i', cp_u_inf, self._u_n)-aL0)

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
        C_L = np.zeros(self._N)
        C_La = np.zeros(self._N)
        alpha = np.zeros(self._N)
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

            np.arctan2(v_ni, v_ai, out=alpha)

            index = 0

            # Loop through airplanes
            for i, airplane_name in enumerate(self._airplane_names):
                airplane_object = self.airplanes[airplane_name]

                # Loop through segments
                for segment_name in self._segment_names[i]:
                    segment_object = airplane_object.wing_segments[segment_name]
                    num_cps = segment_object._N
                    cur_slice = slice(index, index+num_cps)

                    # Get lift coefficient
                    C_L[cur_slice] = segment_object.get_cp_CL(alpha[cur_slice])
                    C_La[cur_slice] = segment_object.get_cp_CLa()

                    index += num_cps

            # Intermediate calcs
            V_i = np.sqrt(np.einsum('ij,ij->i', v_i, v_i))
            w_i = np.cross(v_i, self._dl)
            w_i_mag = np.sqrt(np.einsum('ij,ij->i', w_i, w_i))
            v_iji = np.einsum('ijk,ijk->ij', v_i[:,np.newaxis,:], self._V_ji_trans)

            # Residual vector
            R = 2*w_i_mag*self._Gamma-V_i**2*C_L*self._dS

            error = np.sqrt(np.sum(R**2))

            # Caclulate Jacobian
            J[:,:] = (2*self._Gamma/w_i_mag)[:,np.newaxis]*(np.einsum('ijk,ijk->ij', w_i[:,np.newaxis,:], np.cross(self._V_ji_trans, self._dl)))
            J[:,:] -= (2*self._dS*C_L)[:,np.newaxis]*v_iji
            J[:,:] -= (V_i**2*self._dS)[:,np.newaxis]*(C_La[:,np.newaxis]*(
                v_ai[:,np.newaxis]*np.einsum('ijk,ijk->ij', self._V_ji_trans, self._u_n[:,np.newaxis])-v_ni[:,np.newaxis]*
                np.einsum('ijk,ijk->ij', self._V_ji_trans, self._u_a[:,np.newaxis]))/
                (v_ni**2+v_ai**2)[:,np.newaxis])

            diag_ind = np.diag_indices(self._N)
            J[diag_ind] += 2*w_i_mag

            # Solve for change in gamma
            dGamma = np.linalg.solve(J, -R)

            # Update gammas
            self._Gamma += self._solver_relaxation*dGamma

            if verbose: print("{0:<20}{1:<20}".format(iteration, error))

            if iteration >= self._max_solver_iterations:
                print("Nonlinear solver failed to converge within the allowable number of iterations.")
                break
        else:
            if verbose: print("Nonlinear solver successfully converged.")

        end_time = time.time()
        return end_time-start_time


    def _integrate_forces_and_moments(self, verbose=False):
        # Determines the forces and moments on each lifting surface
        start_time = time.time()

        # Calculate force differential elements
        induced_vels = self._Gamma[:,np.newaxis,np.newaxis]*self._V_ji
        v = self._cp_v_inf+np.sum(induced_vels, axis=0)
        dF_inv = (self._rho*self._Gamma)[:,np.newaxis]*np.cross(v, self._dl)

        # Calculate conditions for determining viscid contributions
        V = np.sqrt(np.einsum('ij,ij->i', v, v))
        u = v/V[:,np.newaxis]
        alpha = np.arctan2(np.einsum('ij,ij->i', v, self._u_n), np.einsum('ij,ij->i', v, self._u_a))
        q_inf = 0.5*self._rho*V**2
        
        # Calculate moment differential elements due to vortex lift
        dM_vortex = np.cross(self._PC, dF_inv)

        index = 0

        self._FM = {}

        # Loop through airplanes
        for i, airplane_name in enumerate(self._airplane_names):
            airplane_object = self.airplanes[airplane_name]
            FM_inv_airplane_total = np.zeros(9)
            FM_vis_airplane_total = np.zeros(9)
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
                "total" : {
                    "FL" : {},
                    "FD" : {},
                    "FS" : {},
                    "Fx" : {},
                    "Fy" : {},
                    "Fz" : {},
                    "Mx" : {},
                    "My" : {},
                    "Mz" : {}
                }
            }

            # Determine freestream vector
            v_trans = airplane_object.get_v_inf()
            v_wind = _quaternion_transform(airplane_object.q, self._get_wind(airplane_object.p_bar))
            v_inf = v_trans + v_wind
            u_inf = (v_inf/np.linalg.norm(v_inf)).flatten()

            # Loop through segments
            for segment_name in self._segment_names[i]:
                num_cps = airplane_object.wing_segments[segment_name]._N
                cur_slice = slice(index, index+num_cps)

                # Determine viscid force
                CD = self.airplanes[airplane_name].wing_segments[segment_name].get_cp_CD(alpha[cur_slice])
                dD = q_inf[cur_slice]*self._dS[cur_slice]*CD
                dF_b_visc = dD[:,np.newaxis]*u[cur_slice]
                F_b_visc = np.sum(dF_b_visc, axis=0)
                self._FM[airplane_name]["viscous"]["Fx"][segment_name] = F_b_visc[0].item()
                self._FM[airplane_name]["viscous"]["Fy"][segment_name] = F_b_visc[1].item()
                self._FM[airplane_name]["viscous"]["Fz"][segment_name] = F_b_visc[2].item()

                L_visc, D_visc, S_visc = self._rotate_aero_forces(F_b_visc, u_inf)
                self._FM[airplane_name]["viscous"]["FL"][segment_name] = L_visc
                self._FM[airplane_name]["viscous"]["FD"][segment_name] = D_visc
                self._FM[airplane_name]["viscous"]["FS"][segment_name] = S_visc

                # Inviscid
                F_b_inv = np.sum(dF_inv[cur_slice], axis=0)
                self._FM[airplane_name]["inviscid"]["Fx"][segment_name] = F_b_inv[0].item()
                self._FM[airplane_name]["inviscid"]["Fy"][segment_name] = F_b_inv[1].item()
                self._FM[airplane_name]["inviscid"]["Fz"][segment_name] = F_b_inv[2].item()

                L_inv, D_inv, S_inv = self._rotate_aero_forces(F_b_inv, u_inf)
                self._FM[airplane_name]["inviscid"]["FL"][segment_name] = L_inv
                self._FM[airplane_name]["inviscid"]["FD"][segment_name] = D_inv
                self._FM[airplane_name]["inviscid"]["FS"][segment_name] = S_inv

                # Determine viscid moment
                Cm = self.airplanes[airplane_name].wing_segments[segment_name].get_cp_Cm(alpha[cur_slice])
                dM_visc = dD[:,np.newaxis]*np.cross(self._PC[cur_slice], u[cur_slice])
                M_b_visc = np.sum(dM_visc, axis=0)
                self._FM[airplane_name]["viscous"]["Mx"][segment_name] = M_b_visc[0].item()
                self._FM[airplane_name]["viscous"]["My"][segment_name] = M_b_visc[1].item()
                self._FM[airplane_name]["viscous"]["Mz"][segment_name] = M_b_visc[2].item()

                # Determine inviscid moment
                dM_section = -(q_inf[cur_slice]*self._dS[cur_slice]*self._c_bar[cur_slice]*Cm)[:,np.newaxis]*self._u_s[cur_slice] # Due to section moment coef
                M_b_inv = np.sum(dM_section+dM_vortex[cur_slice], axis=0)
                self._FM[airplane_name]["inviscid"]["Mx"][segment_name] = M_b_inv[0].item()
                self._FM[airplane_name]["inviscid"]["My"][segment_name] = M_b_inv[1].item()
                self._FM[airplane_name]["inviscid"]["Mz"][segment_name] = M_b_inv[2].item()

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

            self._FM[airplane_name]["inviscid"]["FL"]["total"] = FM_inv_airplane_total[0].item()
            self._FM[airplane_name]["inviscid"]["FD"]["total"] = FM_inv_airplane_total[1].item()
            self._FM[airplane_name]["inviscid"]["FS"]["total"] = FM_inv_airplane_total[2].item()
            self._FM[airplane_name]["inviscid"]["Fx"]["total"] = FM_inv_airplane_total[3].item()
            self._FM[airplane_name]["inviscid"]["Fy"]["total"] = FM_inv_airplane_total[4].item()
            self._FM[airplane_name]["inviscid"]["Fz"]["total"] = FM_inv_airplane_total[5].item()
            self._FM[airplane_name]["inviscid"]["Mx"]["total"] = FM_inv_airplane_total[6].item()
            self._FM[airplane_name]["inviscid"]["My"]["total"] = FM_inv_airplane_total[7].item()
            self._FM[airplane_name]["inviscid"]["Mz"]["total"] = FM_inv_airplane_total[8].item()

            self._FM[airplane_name]["viscous"]["FL"]["total"] = FM_vis_airplane_total[0].item()
            self._FM[airplane_name]["viscous"]["FD"]["total"] = FM_vis_airplane_total[1].item()
            self._FM[airplane_name]["viscous"]["FS"]["total"] = FM_vis_airplane_total[2].item()
            self._FM[airplane_name]["viscous"]["Fx"]["total"] = FM_vis_airplane_total[3].item()
            self._FM[airplane_name]["viscous"]["Fy"]["total"] = FM_vis_airplane_total[4].item()
            self._FM[airplane_name]["viscous"]["Fz"]["total"] = FM_vis_airplane_total[5].item()
            self._FM[airplane_name]["viscous"]["Mx"]["total"] = FM_vis_airplane_total[6].item()
            self._FM[airplane_name]["viscous"]["My"]["total"] = FM_vis_airplane_total[7].item()
            self._FM[airplane_name]["viscous"]["Mz"]["total"] = FM_vis_airplane_total[8].item()

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

        u_lift = np.cross(u_inf,[0.,1.,0.])
        u_side = np.cross(u_lift,u_inf)

        D = np.dot(F, u_inf)
        D_vec = D*u_inf
        
        L = np.dot(F, u_lift)
        L_vec = L*u_lift
        
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
            If this is set to True, the results will be returned as nondimensional coefficients.
            Defaults to False.

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
        integrate_time = self._integrate_forces_and_moments(verbose=verbose)

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

        return self._FM


    def display_wireframe(self, show_legend=False):
        """Displays a 3D wireframe plot of the scene.

        Parameters
        ----------
        show_legend : bool
            If this is set to True, a legend will appear detailing which color corresponds to which wing segment
        """

        segment_names = []

        fig = plt.figure(figsize=plt.figaspect(1.0))
        ax = fig.gca(projection='3d')
        axis_min = 0.0
        axis_max = 0.0

        # Loop through airplanes
        for airplane_name, airplane_object in self.airplanes.items():

            # Loop through segments
            for segment_name, segment_object in airplane_object.wing_segments.items():
                segment_names.append(segment_name)

                points = segment_object.get_outline_points()
                ax.plot(points[:,0], points[:,1], points[:,2], '-')

                axis_max = max([axis_max, max(points.flatten())])
                axis_min = min([axis_min, min(points.flatten())])


        if show_legend:
            ax.legend(segment_names)

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        ax.set_xlim3d(axis_min, axis_max)
        ax.set_ylim3d(axis_min, axis_max)
        ax.set_zlim3d(axis_min, axis_max)

        plt.show()