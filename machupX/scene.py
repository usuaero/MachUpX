from .helpers import _check_filepath, _convert_units, _vectorized_convert_units, _import_value, _quaternion_transform, _quaternion_inverse_transform
from .airplane import Airplane

import json
from skaero.atmosphere import coesa
import numpy as np
import scipy.interpolate as sinterp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class Scene:
    """A class defining a scene containing one or more aircraft.

    Parameters
    ----------
    input_filename : string
        Path to the JSON object specifying the scene parameters. For information on creating
        input files, see examples/How_To_Create_Input_Files.

    Raises
    ------
    IOError
        If input filepath or filename is invalid

    Examples
    --------

    """

    def __init__(self, input_filename):

        self.airplanes = {}
        self._N = 0

        _check_filepath(input_filename,".json")
        self._load_params(input_filename)


    def _load_params(self, input_filename):
        # Loads JSON object and stores input parameters and aircraft
        with open(input_filename) as input_json_handle:
            self._input_dict = json.load(input_json_handle)

        # Store solver parameters
        solver_params = self._input_dict.get("solver",{})
        self._nonlinear_solver = solver_params.get("type", "linear") == "nonlinear"
        self._solver_convergence = solver_params.get("convergence",1e-10)
        self._solver_relaxation = solver_params.get("relaxation",0.9)

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


    def _solve(self):
        # Determines the vortex strengths of all horseshoe vortices in the scene and integrates

        # Start with linear solver
        start_time = time.time()

        # Gather necessary variables
        airplanes = []
        segments = []

        # Atmosphere
        rho = np.zeros(self._N)

        # Geometry
        c_bar = np.zeros(self._N)
        dS = np.zeros(self._N)
        PC = np.zeros((self._N,3))
        P0 = np.zeros((self._N,3))
        P1 = np.zeros((self._N,3))
        u_a = np.zeros((self._N,3))
        u_n = np.zeros((self._N,3))

        # Airfoil parameters
        CLa = np.zeros(self._N)
        aL0 = np.zeros(self._N)

        # Velocities at vortex nodes
        P0_v_inf = np.zeros((self._N,3))
        P1_v_inf = np.zeros((self._N,3))

        # Velocities at control points
        cp_v_inf = np.zeros((self._N,3))

        index = 0

        # Loop through airplanes
        for i, (airplane_name, airplane_object) in enumerate(self.airplanes.items()):
            airplanes.append(airplane_name)
            segments.append([])

            # Loop through segments
            for segment_name, segment_object in airplane_object._wing_segments.items():
                segments[i].append(segment_name)
                num_cps = segment_object._N
                cur_slice = slice(index, index+num_cps)

                # Geometries
                body_cp_locs = segment_object.get_cp_locs()
                PC[cur_slice,:] = body_cp_locs
                c_bar[cur_slice] = segment_object.get_cp_avg_chord_lengths()
                dS[cur_slice] = segment_object.get_array_of_dS()

                node_points = segment_object.get_node_locs()
                P0[cur_slice,:] = node_points[:-1,:]
                P1[cur_slice,:] = node_points[1:,:]

                u_a[cur_slice,:] = segment_object.get_cp_axial_vecs()
                u_n[cur_slice,:] = segment_object.get_cp_normal_vecs()

                # Airfoil parameters
                CLa[cur_slice] = segment_object.get_cp_CLa()
                aL0[cur_slice] = segment_object.get_cp_aL0()

                # Freestream velocity at control points
                # Due to aircraft motion
                if airplane_object.state_type == "aerodynamic":
                    v_trans = airplane_object.v
                else:
                    v_trans = -_quaternion_transform(airplane_object.q, airplane_object.v)

                # Due to wind
                global_cp_locs = airplane_object.p_bar + _quaternion_inverse_transform(airplane_object.q, body_cp_locs)
                cp_v_wind = _quaternion_transform(airplane_object.q, self._get_wind(global_cp_locs))

                # Due to aircraft rotation
                cp_v_rot = -np.cross(airplane_object.w, body_cp_locs)

                cp_v_inf[cur_slice,:] = v_trans+cp_v_wind+cp_v_rot

                # Atmospheric density
                rho[cur_slice] = self._get_density(global_cp_locs)

                # Freestream velocity at vortex nodes
                # Due to wind
                body_node_locs = segment_object.get_node_locs()
                global_node_locs = airplane_object.p_bar + _quaternion_inverse_transform(airplane_object.q, body_node_locs)
                node_v_wind = _quaternion_transform(airplane_object.q, self._get_wind(global_node_locs))

                # Due to aircraft rotation
                node_v_rot = -np.cross(airplane_object.w, body_node_locs)

                node_v_inf = v_trans+node_v_wind+node_v_rot
                P0_v_inf[cur_slice,:] = node_v_inf[:-1,:]
                P1_v_inf[cur_slice,:] = node_v_inf[1:,:]

                index += num_cps

        # Differential length vectors
        dl = P1 - P0

        # Control point velocities
        cp_V_inf = np.linalg.norm(cp_v_inf, axis=1)
        cp_u_inf = cp_v_inf/cp_V_inf[:,np.newaxis]

        # Vortex node velocities
        P0_V_inf = np.linalg.norm(P0_v_inf, axis=1)
        P0_u_inf = P0_v_inf/P0_V_inf[:,np.newaxis]

        P1_V_inf = np.linalg.norm(P1_v_inf, axis=1)
        P1_u_inf = P1_v_inf/P1_V_inf[:,np.newaxis]

        # Spatial node vectors
        r0 = PC[:,np.newaxis] - P0
        r1 = PC[:,np.newaxis] - P1
        r0_mag = np.sqrt(np.einsum('ijk,ijk->ij', r0, r0))
        r1_mag = np.sqrt(np.einsum('ijk,ijk->ij', r1, r1))
        r0_r1_mag = r0_mag*r1_mag

        # Influence of vortex segment 0
        denom = (r0_mag*(r0_mag-np.einsum('ijk,ijk->ij', P0_u_inf[:,np.newaxis], r0)))
        V_ji_due_to_0 = -np.cross(P0_u_inf, r0)/denom[:,:,np.newaxis]

        # Influence of vortex segment 1
        denom = (r1_mag*(r1_mag-np.einsum('ijk,ijk->ij', P1_u_inf[:,np.newaxis], r1)))
        V_ji_due_to_1 = np.cross(P1_u_inf, r1)/denom[:,:,np.newaxis]

        # Influence of bound vortex segment
        with np.errstate(divide='ignore', invalid='ignore'):
            denom = (r0_mag*r1_mag)[:,:,np.newaxis]*((r0_mag*r1_mag)[:,:,np.newaxis]+np.cross(r0, r1))
            V_ji_due_to_bound = ((r0_mag+r1_mag)[:,:,np.newaxis]*np.cross(r0, r1))/denom
            diag_ind = np.diag_indices(self._N)
            V_ji_due_to_bound[diag_ind] = 0.0 # Ensure this actually comes out to be zero, as it should

        V_ji = 1/(4*np.pi)*(V_ji_due_to_0 + V_ji_due_to_bound + V_ji_due_to_1)

        # A matrix
        A = np.zeros((self._N,self._N))
        A[:,:] = -(cp_V_inf**2)[:,np.newaxis]*c_bar[:,np.newaxis]*CLa[:,np.newaxis]*dS[:np.newaxis]*np.einsum('ijk,ijk->ij', V_ji, u_n[:,np.newaxis])
        diag_ind = np.diag_indices(self._N)
        v_inf_cross_dl = np.cross(cp_v_inf, dl)
        A[diag_ind] += 2*np.sqrt(np.einsum('ij,ij->i', v_inf_cross_dl, v_inf_cross_dl))

        # b vector
        b = cp_V_inf**2*c_bar*CLa*(np.einsum('ij,ij->i', cp_v_inf, u_n)-aL0)*dS

        # Solve
        self._Gamma = np.linalg.solve(A, b)

        # Nonlinear improvement
        if self._nonlinear_solver:
            pass

        end_time = time.time()
        print("Execution time: {0} s".format(end_time-start_time))

        # Calculate force differential elements
        induced_vels = (self._Gamma/c_bar)[np.newaxis,:,np.newaxis]*V_ji
        V = cp_v_inf*np.sum(induced_vels, axis=1)
        dF = (rho*self._Gamma)[:,np.newaxis]*np.cross(V, dl)

        # Calculate moment differential elements

        index = 0

        FM = {}

        # Loop through airplanes
        for i, airplane_name in enumerate(airplanes):
            FM_airplane_total = np.zeros(6)
            FM[airplane_name] = {
                "inviscid" : {
                    "Fx" : {},
                    "Fy" : {},
                    "Fz" : {},
                    "Mx" : {},
                    "My" : {},
                    "Mz" : {}
                }
            }

            # Loop through segments
            for segment_name in segments[i]:
                num_cps = self.airplanes[airplane_name]._wing_segments[segment_name]._N
                cur_slice = slice(index, index+num_cps)

                F_segment = np.sum(dF[cur_slice], axis=0)
                FM[airplane_name]["inviscid"]["Fx"][segment_name] = F_segment[0].item()
                FM[airplane_name]["inviscid"]["Fy"][segment_name] = F_segment[1].item()
                FM[airplane_name]["inviscid"]["Fz"][segment_name] = F_segment[2].item()
                FM_airplane_total[:3] += F_segment
                
                index += num_cps

            FM[airplane_name]["inviscid"]["Fx"]["total"] = FM_airplane_total[0].item()
            FM[airplane_name]["inviscid"]["Fy"]["total"] = FM_airplane_total[1].item()
            FM[airplane_name]["inviscid"]["Fz"]["total"] = FM_airplane_total[2].item()

        print(json.dumps(FM, indent=4))


    def solve_forces(self, non_dimensional=False):
        """Solves the NLL equations to determine the forces and moments on the aircraft.

        Parameters
        ----------
        non_dimensional : bool
            If this is set to True, the results will be returned as nondimensional coefficients.
            Defaults to False.

        Returns
        -------
        dict:
            Dictionary of forces and moments acting on each wing segment.
        """
        self._solve()


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
            for segment_name, segment_object in airplane_object._wing_segments.items():
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