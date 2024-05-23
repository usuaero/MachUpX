"""Contains classes and methods for handling airplanes in MachUpX."""

import json
import copy

import matplotlib.pyplot as plt
import numpy as np
import math as m
import scipy.integrate as integ

from airfoil_db import Airfoil
from stl import mesh
from mpl_toolkits.mplot3d import Axes3D
from machupX.helpers import import_value,  euler_to_quat, check_filepath, quat_trans, quat_inv_trans, quat_conj
from machupX.wing_segment import WingSegment


class Airplane:
    """A class defining an airplane. Note the velocity of the airplane is stored
    internally in Earth-fixed coordinates.

    Parameters
    ----------
    name : string
        Name of the airplane.

    airplane_input : string or dict
        Path to the JSON object describing the airplane or dictionary
        containing the same information.

    unit_system : str
        Unit system to use for the aircraft. Can be "English" or "SI".

    scene : machupX.Scene
        Instance of the MUX scene this aircraft belongs to.

    state : dict
        Dictionary describing the initial state vector of the airplane.

    control_state : dict
        Dictionary describing the initial state of the airplane's controls.

    v_wind : ndarray
        Vector giving the wind velocity in Earth-fixed coordinates at the 
        aircraft's center of gravity.

    Returns
    -------
    Airplane
        Returns a newly create airplane object.

    Raises
    ------
    IOError
        If the input filepath or filename is invalid.
    """

    def __init__(self, name, airplane_input, unit_system, scene, init_state={}, init_control_state={}, v_wind=[0.0, 0.0, 0.0]):

        # Store basic info
        self.name = name
        self._unit_sys = unit_system
        self._scene = scene
        
        # Initialize storage
        self.wing_segments = {}
        self._airfoil_database = {}
        self.N = 0

        # Set up data
        self._load_params(airplane_input)
        self.set_state(**init_state, v_wind=v_wind)
        self._create_airfoil_database()
        self._create_origin_segment()
        self._load_wing_segments()
        self._check_reference_params()
        self._initialize_controls(init_control_state)


    def _load_params(self, airplane_input):
        # Parses basic parameters from the airplane input

        # Get input
        if isinstance(airplane_input, str):
            
            # Load JSON object
            check_filepath(airplane_input, ".json")
            with open(airplane_input) as json_handle:
                self._input_dict = json.load(json_handle)

        elif isinstance(airplane_input, dict):
            self._input_dict = airplane_input

        else:
            raise IOError("{0} is not an allowed airplane input type. Must be path or dictionary.".format(type(airplane_input)))

        # Set airplane global params
        self.CG = import_value("CG", self._input_dict, self._unit_sys, [0.0, 0.0, 0.0])
        self.W = import_value("weight", self._input_dict, self._unit_sys, None)
        self.S_w = import_value("area", self._input_dict.get("reference", {}), self._unit_sys, -1)
        self.l_ref_lon = import_value("longitudinal_length", self._input_dict.get("reference", {}), self._unit_sys, -1)
        self.l_ref_lat = import_value("lateral_length", self._input_dict.get("reference", {}), self._unit_sys, -1)


    def set_state(self, **kwargs):
        """Sets the state of the aircraft from the provided dictionary.

        Parameters
        ----------
        position : list, optional
            Earth-fixed position. Defaults to [0.0, 0.0, 0.0].

        velocity : float or list
            Aircraft body-fixed velocity.

        alpha : float, optional
            Aircraft angle of attack. Defaults to 0.0.

        beta : float, optional
            Aircraft sideslip angle. Defaults to 0.0.

        orientation : list, optional
            3-element vector given in the aircraft Euler angles or 4-element vector giving the
            aircraft orientation quaternion. Defaults to [0.0, 0.0, 0.0] or [1.0, 0.0, 0.0, 0.0].

        angular_rates : list, optional
            Body-fixed rotation rates. Defaults to [0.0, 0.0, 0.0].

        angular_rate_frame : str, optional
            Coordinate frame in which the given angular rates are specified. Can be "body", "stab"
            (stability coordinates), or "wind". Defaults to "body".

        v_wind : list, optional
            Local wind vector. Defaults to [0.0, 0.0, 0.0].
        """

        # Get kwargs
        v_wind = kwargs.get("v_wind", [0.0, 0.0, 0.0])

        # Check for ussers using depreciated state definitions
        self.state_type = import_value("type", kwargs, self._unit_sys, "none")
        if self.state_type == 'rigid_body':
            raise IOError("Use of 'rigid_body' state type is no longer supported and the 'state_type' key in general is depreciated. See documentation for details.")

        # Get position and angular rates
        self.p_bar = import_value("position", kwargs, self._unit_sys, [0.0, 0.0, 0.0])

        # Set up orientation quaternion
        self.q = import_value("orientation", kwargs, self._unit_sys, [1.0, 0.0, 0.0, 0.0]) # Default aligns the aircraft with the Earth-fixed coordinates

        if self.q.shape[0] == 3: # Euler angles
            self.q = euler_to_quat(np.radians(self.q))

        elif self.q.shape[0] == 4: # Quaternion
            # Check magnitude
            self.q = self.q/np.linalg.norm(self.q)

        else:
            raise IOError("{0} is not an allowable orientation definition.".format(self.q))

        # Set up velocity
        v_value = import_value("velocity", kwargs, self._unit_sys, None)

        # Velocity magnitude
        if isinstance(v_value, float):

            # Get alpha and beta
            alpha = import_value("alpha", kwargs, self._unit_sys, 0.0)
            beta = import_value("beta", kwargs, self._unit_sys, 0.0)

            # Set state
            self.v = np.array([9.181994, 0.0, 0.0]) # Keeps the following call to set_aerodynamic_state() from breaking; will be overwritten
            self.set_aerodynamic_state(alpha=alpha, beta=beta, velocity=v_value, v_wind=v_wind)

        # Body-fixed velocity vector
        elif isinstance(v_value, np.ndarray) and v_value.shape == (3,):

            # Make sure alpha and beta haven't also been given
            if "alpha" in list(kwargs.keys()) or "beta" in list(kwargs.keys()):
                raise IOError("Alpha and beta are not allowed when the freestream velocity is a vector.")

            # Store
            self.v = quat_inv_trans(self.q, v_value)

        else:
            raise IOError("{0} is not an allowable velocity definition.".format(v_value))

        # Set angular rates
        w_raw = import_value("angular_rates", kwargs, self._unit_sys, [0.0, 0.0, 0.0])
        self.angular_rate_frame = kwargs.get("angular_rate_frame", "body")

        if self.angular_rate_frame == "body": # Body-fixed
            self.w = w_raw

        elif self.angular_rate_frame == "stab": # Stability coordinates
            try:
                self.q_to_stab = quat_conj(euler_to_quat([0.0, m.radians(alpha), 0.0]))
            except:
                alpha = m.atan2(v_value[2], v_value[0])
                self.q_to_stab = quat_conj(euler_to_quat([0.0, alpha, 0.0]))
            self.w = quat_inv_trans(self.q_to_stab, w_raw)

        elif self.angular_rate_frame == "wind": # Wind frame
            try:
                self.q_to_wind = quat_conj(euler_to_quat([0.0, m.radians(alpha), m.radians(beta)]))
            except:
                alpha = m.atan2(v_value[2], v_value[0])
                beta = m.asin(v_value[1]/m.sqrt(v_value[0]**2+v_value[1]**2+v_value[2]**2))
                self.q_to_wind = quat_conj(euler_to_quat([0.0, alpha, beta]))
            self.w = quat_inv_trans(self.q_to_wind, w_raw)

        else:
            raise IOError("{0} is not an allowable angular rate frame.".format(self.angular_rate_frame))


    def get_state(self):
        """Returns the aircraft's current state vector.

        Returns
        -------
        ndarray
            Current Earth-fixed velocity vector.

        ndarray
            Current body-fixed angular velocity vector.

        ndarray
            Current Earth-fixed position vector.

        ndarray
            Current active orientation quaternion.
        """

        return np.copy(self.v), np.copy(self.w), np.copy(self.p_bar), np.copy(self.q)


    def get_aerodynamic_state(self, v_wind=[0.0, 0.0, 0.0]):
        """Returns the aircraft's angle of attack, sideslip angle, and freestream velocity magnitude.
        Assumes a bank angle of zero.

        Parameters
        ----------
        v_wind : ndarray
            The local wind vector at the aircraft body-fixed origin in flat-earth 
            coordinates. Defaults to [0.0, 0.0, 0.0].

        Returns
        -------
        alpha : float
            Angle of attack in degrees

        beta : float
            Sideslip angle in degrees

        velocity : float
            Magnitude of the freestream velocity
        """
        # Determine velocity relative to the wind in the body-fixed frame
        v = quat_trans(self.q, self.v-v_wind)
        V = m.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])

        # Calculate values
        alpha = m.degrees(m.atan2(v[2], v[0]))
        beta = m.degrees(m.asin(v[1]/V))
        return alpha, beta, V


    def set_aerodynamic_state(self, **kwargs):
        """Sets the velocity of the aircraft so that its angle of attack and sideslip
        angle are what is desired. Scales and rotates the aircraft's velocity to match
        what is desired. DOES NOT CHANGE AIRCRAFT ORIENTATION.

        Parameters
        ----------
        alpha : float
            Desired angle of attack in degrees. Defaults to current angle of attack.

        beta : float
            Desired sideslip angle in degrees. Defaults to current sideslip angle.

        velocity : float
            Magnitude of the freestream velocity as seen by the aircraft. Defaults to the
            current freestream velocity.

        v_wind : ndarray
            The local wind vector at the aircraft body-fixed origin in flat-earth 
            coordinates. Defaults to [0.0, 0.0, 0.0].
        """

        # Determine the current state
        v_wind = kwargs.get("v_wind", [0.0, 0.0, 0.0])
        current_aero_state = self.get_aerodynamic_state(v_wind=v_wind)

        # Determine the desired state
        alpha = kwargs.get("alpha", current_aero_state[0])
        beta = kwargs.get("beta", current_aero_state[1])
        velocity = kwargs.get("velocity", current_aero_state[2])

        # Convert to flank angle
        C_a = m.cos(m.radians(alpha))
        B_f = m.atan(m.tan(m.radians(beta))/C_a)

        # Calculate trigonometric values
        S_a = m.sin(m.radians(alpha))
        C_B = m.cos(B_f)
        S_B = m.sin(B_f)

        # Determine freestream velocity components in body-fixed frame (Mech of Flight Eqs. 7.1.10-12)
        v_inf_b = np.zeros(3)
        denom = 1/m.sqrt(1-S_a*S_a*S_B*S_B)
        v_inf_b[0] = velocity*C_a*C_B*denom
        v_inf_b[1] = velocity*C_a*S_B*denom
        v_inf_b[2] = velocity*S_a*C_B*denom

        # Transform to earth-fixed coordinates
        self.v = v_wind+quat_inv_trans(self.q, v_inf_b)


    def _initialize_controls(self, init_control_state):
        # Initializes the control surfaces on the airplane

        # Store control parameters
        self._control_symmetry = {}
        self.control_names = []
        self.current_control_state = {}
        controls = self._input_dict.get("controls", {})
        for key in controls:
            self.control_names.append(key)
            self._control_symmetry[key] = controls[key].get("is_symmetric", True)
            self.current_control_state[key] = 0.0

        # Set controls
        self.set_control_state(control_state=init_control_state)
        

    def _create_origin_segment(self):
        # Create a wing segment which has no properties but which other segments 
        # connect to.
        origin_dict = {
            "ID" : 0,
            "is_main" : False
        }
        self._origin_segment = WingSegment("origin", origin_dict, "both", self._unit_sys, self._airfoil_database)

    
    def add_wing_segment(self, wing_segment_name, input_dict, recalculate_geometry=True):
        """Adds a wing segment to the airplane.

        Parameters
        ----------
        wing_segment_name : str
            Name of the wing segment.

        input_dict : dict
            Dictionary describing the wing segment. Same as specified for input files.

        recalculate_geometry : boolean, optional
            Defaults to True. SHOULD NOT BE SET TO FALSE. This is required for internal functionality.

        Returns
        -------

        Raises
        ------
        IOError
            If the input is improperly specified.
        """

        # Let me take a moment to explain the structure of wing segments in MachUpX. This is
        # for the sake of other developers. The way we have decided to define wing segments 
        # makes them fall very naturally into a tree-type structure. Any given wing segment 
        # is attached (we use this term loosely; more accurately, the position of one wing 
        # segment is defined relative to another) to another wing segment or the origin. 
        # Eventually, these all lead back to the origin. The origin here is a "dummy" wing 
        # segment which has no other properties than an ID of 0. Adding a wing segment is done
        # recursively via the tree. Each wing segment knows which wing segments attach to it.
        # However, no wing segment knows who it attaches to, only the location of its origin. 

        # The tree structure makes certain operations, such as integrating forces and moments 
        # and applying structural deformations, very natural. However, generating the lifting-
        # line matrix equations from this structure is very cumbersome. Therefore, we also 
        # store references to each wing segment at the Airplane level in a list. This makes 
        # generating the lifting-line matrix much more friendly. This makes the code a little 
        # more fragile, but this is Python and we assume the user is being responsible.
        
        if wing_segment_name in self.wing_segments.keys():
            raise IOError("Wing segment {0} already exists in this airplane.".format(wing_segment_name))

        side = input_dict.get("side", "both")
        if not (side == "left" or side == "right" or side == "both"):
            raise IOError("{0} is not a proper side designation.".format(side))

        if side == "left" or side == "both":
            self.wing_segments[wing_segment_name+"_left"] = self._origin_segment.attach_wing_segment(wing_segment_name+"_left", input_dict, "left", self._unit_sys, self._airfoil_database)
            self.N += self.wing_segments[wing_segment_name+"_left"].N

        if side == "right" or side == "both":
            self.wing_segments[wing_segment_name+"_right"] = self._origin_segment.attach_wing_segment(wing_segment_name+"_right", input_dict, "right", self._unit_sys, self._airfoil_database)
            self.N += self.wing_segments[wing_segment_name+"_right"].N

        if recalculate_geometry:
            self._calculate_geometry()


    def _load_wing_segments(self):
        # Reads in the wing segments from the input dict and attaches them

        # Decide in what order to add the segments, since some are defined relative to others
        wing_dict = self._input_dict.get("wings", {})
        seg_names = []
        for key, value in wing_dict.items():

            # Find out if any of the segments already added to the list are dependent upon this one
            curr_ID = value["ID"]
            found_dependent = False
            for i, seg_name in enumerate(seg_names):
                if wing_dict[seg_name].get("connect_to", {}).get("ID", 0) == curr_ID:
                    seg_names.insert(i, key)
                    found_dependent = True
                    break

            # Add the wing segment to the end if nothing is dependent on it
            if not found_dependent:
                seg_names.append(key)

        # Add segments
        for seg_name in seg_names:
            self.add_wing_segment(seg_name, self._input_dict["wings"][seg_name], recalculate_geometry=False)

        # Perform geometry calculations
        self._calculate_geometry()


    def _calculate_geometry(self):
        # Figures out which wing segments are contiguous and sets up the lists of control points and vortex nodes

        # Initialize arrays
        # Geometry
        self.c_bar = np.zeros(self.N) # Average chord
        self.b_seg = np.zeros(self.N) # Segment semispan
        self.P0_chord = np.zeros(self.N)
        self.P1_chord = np.zeros(self.N)
        self.dS = np.zeros(self.N) # Differential planform area
        self.max_camber = np.zeros(self.N)
        self.max_thickness = np.zeros(self.N)

        # Control points
        self.PC = np.zeros((self.N,3)) # Control point location
        self.PC_CG = np.zeros((self.N,3)) # Control point location relative to CG
        self.PC_span_locs = np.zeros(self.N) # Control point span locations

        # Inbound nodes
        self.P0 = np.zeros((self.N,3)) # Inbound vortex node location
        self.P0_eff = np.zeros((self.N,self.N,3)) # Inbound vortex node location (effective locus of aerodynamic centers)
        self.P0_span_locs = np.zeros(self.N)
        self.P0_joint_eff = np.zeros((self.N,self.N,3)) # Inbound vortex joint node location (effective locus of aerodynamic centers)

        # Outbound nodes
        self.P1 = np.zeros((self.N,3)) # Outbound vortex node location
        self.P1_eff = np.zeros((self.N,self.N,3)) # Outbound vortex node location (effective locus of aerodynamic centers)
        self.P1_span_locs = np.zeros(self.N)
        self.P1_joint_eff = np.zeros((self.N,self.N,3)) # Outbound vortex joint node location (effective locus of aerodynamic centers)

        # Section unit vectors
        self.u_a = np.zeros((self.N,3))
        self.u_n = np.zeros((self.N,3))
        self.u_s = np.zeros((self.N,3))
        self.u_a_unswept = np.zeros((self.N,3))
        self.u_n_unswept = np.zeros((self.N,3))
        self.u_s_unswept = np.zeros((self.N,3))
        self.P0_u_a = np.zeros((self.N,3))
        self.P1_u_a = np.zeros((self.N,3))

        # Reid correction parameters
        reid_corr = np.zeros(self.N) # Whether to use Reid corrections
        sigma_blend = np.zeros(self.N) # Blending distance
        delta_joint = np.zeros(self.N) # Joint length

        # Group wing segments into wings
        self._sort_segments_into_wings()

        # Gather segment data
        self._wing_N = []
        self.wing_slices = []
        cur_index = 0
        for i in range(self._num_wings):

            # Reset params for this wing
            self._wing_N.append(0)
            cur_span_from_left_tip = 0.0

            # Loop through segments
            for segment in self._segments_in_wings[i]:

                # Determine slice for this segment
                N = segment.N
                self._wing_N[i] += N
                cur_slice = slice(cur_index, cur_index+N)

                # Store section geometry
                self.c_bar[cur_slice] = segment.c_bar_cp
                self.b_seg[cur_slice] = segment.b
                self.PC[cur_slice,:] = segment.control_points
                self.dS[cur_slice] = segment.dS
                self.P0_chord[cur_slice] = segment.c_node[:-1]
                self.P1_chord[cur_slice] = segment.c_node[1:]
                self.max_camber[cur_slice] = segment.max_camber_cp
                self.max_thickness[cur_slice] = segment.max_thickness_cp

                # Reid-Hunsaker parameters
                reid_corr[cur_slice] = segment.reid_corr
                delta_joint[cur_slice] = segment.delta_joint
                sigma_blend[cur_slice] = (2.0/(segment.b*segment.blend_dist*np.cos(segment.sweep_cp)))**2

                # Let's talk about things. In the G-H journal paper, sigma is derived assuming the blending factor is formulated in terms of a nondimensional distance. However, in MachUpX, the blending factor is formulated in terms of a dimensional distance (since MachUpX has to allow for the possibility of multiple wing segments sharing a liftine line). So, the equation used here includes the wingspan while that given in the G-H paper does not.

                # Store control point and node span locations
                if segment.side == "left":
                    self.PC_span_locs[cur_slice] = cur_span_from_left_tip+(1.0-segment.cp_span_locs)*segment.b
                    node_spans = cur_span_from_left_tip+(1.0-segment.node_span_locs)*segment.b
                else:
                    self.PC_span_locs[cur_slice] = cur_span_from_left_tip+segment.cp_span_locs*segment.b
                    node_spans = cur_span_from_left_tip+segment.node_span_locs*segment.b
                self.P0_span_locs[cur_slice] = node_spans[:-1]
                self.P1_span_locs[cur_slice] = node_spans[1:]

                # Store original node locations, to be updated point by point
                self.P0[cur_slice,:] = segment.nodes[:-1]
                self.P1[cur_slice,:] = segment.nodes[1:]
                self.P0_eff[:,cur_slice,:] = segment.nodes[:-1]
                self.P1_eff[:,cur_slice,:] = segment.nodes[1:]

                # Section direction vectors
                self.u_a[cur_slice,:] = segment.u_a_cp
                self.u_n[cur_slice,:] = segment.u_n_cp
                self.u_s[cur_slice,:] = segment.u_s_cp
                self.u_a_unswept[cur_slice,:] = segment.u_a_cp_unswept
                self.u_n_unswept[cur_slice,:] = segment.u_n_cp_unswept
                self.u_s_unswept[cur_slice,:] = segment.u_s_cp_unswept
                self.P0_u_a[cur_slice,:] = segment.u_a_node[:-1]
                self.P1_u_a[cur_slice,:] = segment.u_a_node[1:]

                # Update for next segment
                cur_index += N
                cur_span_from_left_tip += segment.b

            # Determine slice for this wing
            self.wing_slices.append(slice(cur_index-self._wing_N[i], cur_index))

        # Calculate joint locations for actual lifting line
        # I feel as if there needs to be some blending in u_a here as well. This would make the trailing vortex sheet
        # discontinuous when seen by another lifting surface. Right? But I can't think of a good way to fix this right now,
        # so I'm going to leave it. Have fun ;)
        # Granted, I don't think we ever actually use the actual lifting-line to calculate induced velocities...so probably not
        self.P0_joint = self.P0+(self.P0_chord*delta_joint*reid_corr)[:,np.newaxis]*self.P0_u_a
        self.P1_joint = self.P1+(self.P1_chord*delta_joint*reid_corr)[:,np.newaxis]*self.P1_u_a
        self.P0_joint_eff[:] = np.copy(self.P0_joint)[np.newaxis,:,:]
        self.P1_joint_eff[:] = np.copy(self.P1_joint)[np.newaxis,:,:]

        # Calculate control point locations relative to CG
        self.PC_CG = self.PC-self.CG[np.newaxis,:]

        # Calculate control point derivative with respect to distance along the lifting line projected onto the body y-z plane
        PC_deriv = self.u_s/np.linalg.norm(self.u_s[:,1:], axis=1, keepdims=True)

        # Determine the sweep angle of each section based on the lifting line
        self.section_sweep = -np.arctan(PC_deriv[:,0])

        # Calculate effective lifting lines
        cur_wing = 0
        wing_slice = self.wing_slices[cur_wing]
        for i in range(self.N):

            # Check if we've moved beyond the current wing
            if i >= self.wing_slices[cur_wing].stop:
                cur_wing += 1
                wing_slice = self.wing_slices[cur_wing]

            # General NLL corrections; if this is skipped, the node locations remain unchanged
            if reid_corr[i]:

                # Get control point of interest
                PC = self.PC[i,:]
                PC_span = self.PC_span_locs[i]

                # Blend P0
                ds0 = self.P0_span_locs[wing_slice]-PC_span
                straight_ac = PC+PC_deriv[i,:]*ds0[:,np.newaxis]
                blend_0 = np.exp(-sigma_blend[i]*ds0*ds0)[:,np.newaxis]
                self.P0_eff[i,wing_slice,:] = straight_ac*blend_0+self.P0_eff[i,wing_slice,:]*(1.0-blend_0)

                # Blend P1
                ds1 = self.P1_span_locs[wing_slice]-PC_span
                straight_ac = PC+PC_deriv[i,:]*ds1[:,np.newaxis]
                blend_1 = np.exp(-sigma_blend[i]*ds1*ds1)[:,np.newaxis]
                self.P1_eff[i,wing_slice,:] = straight_ac*blend_1+self.P1_eff[i,wing_slice,:]*(1.0-blend_1)

                # Blend u_a_unswept
                ds = self.PC_span_locs[wing_slice]-PC_span
                blend = np.exp(-sigma_blend[i]*ds*ds)[:,np.newaxis]
                u_a_straight = self.u_a_unswept[i,:]
                u_a = u_a_straight*blend+self.u_a_unswept[wing_slice,:]*(1.0-blend)
                u_a = u_a/np.linalg.norm(u_a, axis=1, keepdims=True)

                # Place vortex joints
                # These equations ensure the joint vector (u_j) is orthogonal to the lifting-line tangent and lies in the plane defined
                # by the lifting-line tangent and the unswept axial vector (i.e. chord line). You get these by solving
                #
                #   < u_j, T > = 0
                #   < u_j, u_a > > 0
                #   u_j = c1*u_a+c2*T
                # 
                # Lead to
                #
                # c1 = sqrt(1/1-k^2)
                # c2 = -c1*k
                #
                # where
                #
                # k = < T, u_a >

                # Same for both
                dt = np.zeros(wing_slice.stop-wing_slice.start)

                # P0
                d_P0 = np.diff(self.P0_eff[i,wing_slice,:], axis=0)
                dt[1:] = np.cumsum(np.linalg.norm(d_P0, axis=1))
                T0 = np.gradient(self.P0_eff[i,wing_slice,:], dt, edge_order=2, axis=0)
                T0 = T0/np.linalg.norm(T0, axis=1)[:,np.newaxis]
                k = np.einsum('ij,ij->i', T0, u_a)
                c1 = np.sqrt(1/(1-k*k))
                c2 = -c1*k
                u_j = c1[:,np.newaxis]*u_a+c2[:,np.newaxis]*T0
                u_j = u_j/np.linalg.norm(u_j, axis=-1, keepdims=True)
                self.P0_joint_eff[i,wing_slice,:] = self.P0_eff[i,wing_slice,:]+self.P0_chord[wing_slice,np.newaxis]*delta_joint[wing_slice,np.newaxis]*u_j

                # P1 joint
                d_P1 = np.diff(self.P1_eff[i,wing_slice,:], axis=0)
                dt[1:] = np.cumsum(np.linalg.norm(d_P1, axis=1))
                T1 = np.gradient(self.P1_eff[i,wing_slice,:], dt, edge_order=2, axis=0)
                T1 = T1/np.linalg.norm(T1, axis=1)[:,np.newaxis]
                k = np.einsum('ij,ij->i', T1, u_a)
                c1 = np.sqrt(1/(1-k*k))
                c2 = -c1*k
                u_j = c1[:,np.newaxis]*u_a+c2[:,np.newaxis]*T1
                u_j = u_j/np.linalg.norm(u_j, axis=-1, keepdims=True)
                self.P1_joint_eff[i,wing_slice,:] = self.P1_eff[i,wing_slice,:]+self.P1_chord[wing_slice,np.newaxis]*delta_joint[wing_slice,np.newaxis]*u_j

                # Plot effective vortices for control points at the root
                if False and i > (self.N//2-2) and i < (self.N//2+1):
                    fig = plt.figure(figsize=plt.figaspect(1.0))
                    ax = fig.gca(projection='3d')
                    for j in range(wing_slice.start, wing_slice.stop):
                        ax.plot([self.P0_joint_eff[i,j,0], self.P0_eff[i,j,0], self.P1_eff[i,j,0], self.P1_joint_eff[i,j,0]],
                                [self.P0_joint_eff[i,j,1], self.P0_eff[i,j,1], self.P1_eff[i,j,1], self.P1_joint_eff[i,j,1]],
                                [self.P0_joint_eff[i,j,2], self.P0_eff[i,j,2], self.P1_eff[i,j,2], self.P1_joint_eff[i,j,2]],
                                '--')

                    lim = np.max(np.max(np.max(self.P0_joint_eff)))
                    ax.set_xlim3d(lim, -lim)
                    ax.set_ylim3d(lim, -lim)
                    ax.set_zlim3d(lim, -lim)
                    plt.show()

            else:

                # Copy node locations into joint locations (i.e. no jointing)
                self.P0_joint_eff[i,:,:] = np.copy(self.P0_eff[i,:,:])
                self.P1_joint_eff[i,:,:] = np.copy(self.P1_eff[i,:,:])

        # Calculate vectors from control points to vortex node locations
        self.r_0 = self.PC[:,np.newaxis,:]-self.P0_eff
        self.r_1 = self.PC[:,np.newaxis,:]-self.P1_eff
        self.r_0_joint = self.PC[:,np.newaxis,:]-self.P0_joint_eff
        self.r_1_joint = self.PC[:,np.newaxis,:]-self.P1_joint_eff

        # Calculate spatial node vector magnitudes
        self.r_0_mag = np.linalg.norm(self.r_0, axis=-1)
        self.r_0_joint_mag = np.linalg.norm(self.r_0_joint, axis=-1)
        self.r_1_mag = np.linalg.norm(self.r_1, axis=-1)
        self.r_1_joint_mag = np.linalg.norm(self.r_1_joint, axis=-1)

        # Calculate magnitude products
        self.r_0_r_0_joint_mag = self.r_0_mag*self.r_0_joint_mag
        self.r_0_r_1_mag = self.r_0_mag*self.r_1_mag
        self.r_1_r_1_joint_mag = self.r_1_mag*self.r_1_joint_mag

        # Calculate differential length vectors
        self.dl = self.P1-self.P0

        # Plot effective LACs
        if self._input_dict.get("plot_lacs", False):
            fig = plt.figure(figsize=plt.figaspect(1.0))
            ax = fig.gca(projection='3d')
            for i in range(self.N):
                for wing_slice in self.wing_slices:
                    ax.plot(self.P0_eff[i,wing_slice,0], self.P0_eff[i,wing_slice,1], self.P0_eff[i,wing_slice,2], 'r-')
                    ax.plot(self.P0_joint_eff[i,wing_slice,0], self.P0_joint_eff[i,wing_slice,1], self.P0_joint_eff[i,wing_slice,2], color='orange')
                    ax.plot(self.P1_eff[i,wing_slice,0], self.P1_eff[i,wing_slice,1], self.P1_eff[i,wing_slice,2], 'b-')
                    ax.plot(self.P1_joint_eff[i,wing_slice,0], self.P1_joint_eff[i,wing_slice,1], self.P1_joint_eff[i,wing_slice,2], 'g-')

            lim = np.max(np.max(np.max(self.P0_joint_eff)))
            ax.set_xlim3d(lim, -lim)
            ax.set_ylim3d(lim, -lim)
            ax.set_zlim3d(lim, -lim)
            plt.show()


    def _sort_segments_into_wings(self):
        # Groups segments into wings for calculating effective lifting lines

        # Count the number of "lifting lines" we have
        N_ll = 0
        originals = [] # Segments that are not continuations
        for segment_name, segment in self.wing_segments.items():

            segment.wing_ID = -1

            # Skip left segments if there is a matching right one with no offset
            #if segment.side == "left" and segment.has_mirror and abs(segment.y_offset) < 1e-12 and not (segment._connected_to_loc == "tip" and np.linalg.norm(segment._delta_origin) > 1.e-12):
            if segment.side == "left" and segment.has_mirror and abs(segment.y_offset) < 1e-12:
                continue

            # Check that this segment is not a continuation
            if not segment.is_continuation():
                N_ll += 1
                originals.append(segment_name)

            # If it is a continuation, check if it's a branch (e.g. a T-tail on top of a vertical stabilizer)
            else:

                # Check this segment has both sides
                if segment.has_mirror:

                    # Check the segment it is attached to does not
                    if not segment.parent_has_mirror:
                        N_ll += 1
                        originals.append(segment_name)

        # Gather wing segments sharing lifting lines
        self._segments_in_wings = []
        i_curr_ll = 0
        for original_segment_name in originals:

            # Get original segment object
            original_segment = self.wing_segments[original_segment_name]
            original_ID = original_segment.ID
            original_segment.wing_ID = i_curr_ll
            original_joined_at_middle = abs(original_segment.y_offset) < 1e-12

            # Add list for this lifting line, initialized with the original segment
            self._segments_in_wings.append([original_segment])

            # Add the original segment's mirror (if there is no offset)
            if original_segment.has_mirror and original_joined_at_middle:
                mirror_segment = self.wing_segments[original_segment_name.replace("right", "left")]
                mirror_segment.wing_ID = i_curr_ll
                self._segments_in_wings[i_curr_ll].append(mirror_segment)

            # Outer loop makes sure we check connections to connected segments, not just the original
            added_segment = True
            while added_segment:
                added_segment = False

                # Loop through other wing segments to see if they are a continuation of this original or another connected segment
                for segment_name, segment in self.wing_segments.items():

                    # Check whether this segment is already in the originals
                    if segment_name in originals:
                        continue

                    # Check whether this segment is already in the list of connected segments
                    if segment.wing_ID != -1:
                        continue

                    # Check continuation
                    if not segment.is_continuation():
                        continue

                    # Check whether this segment is connected to the original segment
                    if segment._connected_to_ID == original_ID:

                        # If the original doesn't have a mirror but the other one does, they will not be on the same lifting line
                        if not original_segment.has_mirror and segment.has_mirror:
                            continue

                        # If the original segment is joined at the middle, then we don't need to check anything else
                        # If not, we need to make sure it's on the same side
                        if original_joined_at_middle or (not original_joined_at_middle and segment.side == original_segment.side):
                            segment.wing_ID = i_curr_ll
                            self._segments_in_wings[i_curr_ll].append(segment)
                            added_segment = True

                    # Check whether this segment is connected to any of the other connected segments
                    else:
                        for connected_segment in self._segments_in_wings[i_curr_ll]:

                            if segment._connected_to_ID == connected_segment.ID:

                                if not connected_segment.has_mirror and segment.has_mirror:
                                    continue

                                # We can assume the connected segment is not joined at the middle
                                if segment.side == connected_segment.side:
                                    segment.wing_ID = i_curr_ll
                                    self._segments_in_wings[i_curr_ll].append(segment)
                                    added_segment = True

            # Increment lifting line index
            i_curr_ll += 1

        # Store number of wings
        self._num_wings = N_ll

        # Sort left to right
        self._sort_segments_left_to_right()


    def _sort_segments_left_to_right(self):
        # Sort segments along wingspan from left to right

        for i in range(self._num_wings):
            sorted_segments = []

            # Sort left segments
            while True:
                next_segment = None
                norm_to_beat = 0.0
                for segment in self._segments_in_wings[i]:
                    if "_left" in segment.name:
                        tip = segment.get_tip_loc()
                        norm = m.sqrt(tip[1]*tip[1]+tip[2]*tip[2])
                        if norm_to_beat < norm and segment not in sorted_segments:
                            next_segment = segment
                            norm_to_beat = norm

                # If nothing new got added to the sorted list, we're done with the left segments
                if next_segment is None:
                    break

                # Add segment to list
                sorted_segments.append(next_segment)

            # Sort right segments
            while True:
                next_segment = None
                norm_to_beat = np.inf
                for segment in self._segments_in_wings[i]:
                    if "_right" in segment.name:
                        tip = segment.get_tip_loc()
                        norm = m.sqrt(tip[1]*tip[1]+tip[2]*tip[2])
                        if norm_to_beat > norm and segment not in sorted_segments:
                            next_segment = segment
                            norm_to_beat = norm

                # If nothing new got added to the sorted list, we're done with the right segments
                if next_segment is None:
                    break

                # Add segment to list
                sorted_segments.append(next_segment)

            self._segments_in_wings[i] = sorted_segments

        # Store segments in single list without separating into wings
        self.segments = []
        for wing in self._segments_in_wings:
            for segment in wing:
                self.segments.append(segment)


    def _check_reference_params(self):
        # If the reference area and lengths have not been set, this takes care of that.

        # Reference area
        if self.S_w == -1:
            self.S_w = 0.0
            for (_, wing_segment) in self.wing_segments.items():
                if wing_segment.is_main:
                    self.S_w += np.sum(wing_segment.dS)

        # Lateral reference length
        if self.l_ref_lat == -1:
            self.l_ref_lat = 0.0
            for (_, wing_segment) in self.wing_segments.items():
                if wing_segment.is_main and wing_segment.side == "right":
                    self.l_ref_lat += wing_segment.b*2.0

        # Longitudinal reference length
        try:
            if self.l_ref_lon == -1:
                self.l_ref_lon = self.S_w/(self.l_ref_lat)
        except:
            raise IOError("No wing was specified as main for {0} so reference parameters cannot be determined.".format(self.name))


    def _get_wing_segment(self, wing_segment_name):
        # Returns a reference to the specified wing segment. Use with caution. Or just don't use.
        return self._origin_segment._get_attached_wing_segment(wing_segment_name)

    
    def _create_airfoil_database(self):
        # Creates a dictionary of all the airfoils. This dictionary is then passed to each 
        # wing segment when it gets created for the wing segment to use.

        airfoils = self._input_dict.get("airfoils", {"default" : {} })

        # Load airfoil database from separate file
        if isinstance(airfoils, str):
            check_filepath(airfoils, ".json")
            with open(airfoils, 'r') as airfoil_db_handle:
                airfoil_dict = json.load(airfoil_db_handle)

        # Load from airplane dict
        elif isinstance(airfoils, dict):
            airfoil_dict = airfoils

        else:
            raise IOError("'airfoils' must be a string or dict.")

        for key in airfoil_dict:
            self._airfoil_database[key] = Airfoil(key, airfoil_dict[key])


    def set_control_state(self, control_state={}):
        """Sets the control surface deflections on the airplane using the control mapping.

        Parameters
        ----------
        control_state : dict
            A set of key-value pairs where the key is the name of the control and the 
            value is the deflection. For positive mapping values, a positive deflection 
            here will cause a downward deflection of symmetric control surfaces and 
            downward deflection of the right surface for anti-symmetric control surfaces.
            Units may be specified as in the input file. Any deflections not given will 
            default to zero; the previous state is not preserved
        """
        # Store controls
        for key,_ in self.current_control_state.items():
            self.current_control_state[key] = control_state.get(key, 0.0)

        # Apply to wing segments
        for _,wing_segment in self.wing_segments.items():
            wing_segment.apply_control(control_state, self._control_symmetry)


    def get_MAC(self):
        """Returns the mean aerodynamic chord (MAC).

        Returns
        -------
        MAC : dict
            MAC data. Structured as

                {
                    "length" : mean aerodynamic chord length,
                    "C_point" : center of constant lift, determined by Eq. 2.6.2 from Nickel and Wohlfahrt "Tailless Aircraft"
                }
        """

        # Loop through main wing segments to calculate MAC length and location
        MAC = 0.0
        MAC_loc = 0.0
        S = 0.0
        for (_, wing_segment) in self.wing_segments.items():
            if wing_segment.is_main:

                if False: # Toggle for exact integral vs approximation
                    # Note for the integral methods, everything is divided by the semispan
                    MAC += integ.quad(lambda s: wing_segment.get_chord(s)**2, 0.0, 1.0)[0] # More exact but gives approximately the same as ^
                    MAC_loc += integ.quad(lambda s: wing_segment.get_chord(s)*wing_segment._get_section_ac_loc(s)[0], 0.0, 1.0)[0]
                    S += integ.quad(lambda s: wing_segment.get_chord(s), 0.0, 1.0)[0]

                else:
                    MAC += np.sum(wing_segment.dS*np.cos(wing_segment.dihedral_cp)*wing_segment.c_bar_cp)
                    MAC_loc += np.sum(wing_segment.dS*np.cos(wing_segment.dihedral_cp)*wing_segment.control_points[:,0])
                    S += np.sum(wing_segment.dS)

        # Divide by planform area
        MAC /= S
        MAC_loc /= S

        # Package results
        results = {
            "length" : MAC,
            "C_point" : MAC_loc
        }
        return results


    def export_stl(self, **kwargs):
        """Exports a .stl model of the aircraft.

        Parameters
        ----------
        filename
            File to export the model to. Must be .stl.

        section_resolution
            Number of points to use in discretizing the airfoil section outlines. Defaults to 200.

        close_te : bool, optional
            Whether to force the trailing edge to be sealed. Defaults to true
        """

        # Check for .stl file
        filename = kwargs.get("filename")
        if ".stl" not in filename:
            raise IOError("{0} is not a .stl file.".format(filename))

        # Loop through segments
        num_facets = 0
        vector_dict = {}
        for segment_name, segment_object in self.wing_segments.items():
            vectors = segment_object.get_stl_vectors(**kwargs)
            vector_dict[segment_name] = vectors
            num_facets += int(vectors.shape[0]/3)

        # Allocate mesh
        model_mesh = mesh.Mesh(np.zeros(num_facets, dtype=mesh.Mesh.dtype))

        # Store vectors
        index = 0
        for segment_name, segment_object in self.wing_segments.items():
            num_segment_facets = int(vector_dict[segment_name].shape[0]/3)
            for i in range(index, index+num_segment_facets):
                for j in range(3):
                    model_mesh.vectors[i][j] = vector_dict[segment_name][3*(i-index)+j]
            index += num_segment_facets

        # Export
        model_mesh.save(filename)


    def export_vtk(self, **kwargs):
        """Exports a .vtk model of the aircraft.

        Parameters
        ----------
        filename
            File to export the model to. Must be .vtk.

        section_resolution
            Number of points to use in discretizing the airfoil section outlines. Defaults to 200.

        close_te : bool, optional
            Whether to force the trailing edge to be sealed. Defaults to True
        """

        # Check for .stl file
        filename = kwargs.get("filename")
        if ".vtk" not in filename:
            raise IOError("{0} is not a .vtk file.".format(filename))

        # Open file
        with open(filename, 'w') as export_handle:
            
            # Write header
            print("# vtk DataFile Version 3.0", file=export_handle)
            print("MachUpX aircraft mesh, USU AeroLab (c) 2020.", file=export_handle)
            print("ASCII", file=export_handle)

            # Write dataset
            print("DATASET POLYDATA", file=export_handle)

            # Write vertices
            vertices, panel_indices = self._get_vtk_data(**kwargs)
            print("POINTS {0} float".format(len(vertices)), file=export_handle)
            for vertex in vertices:
                print("{0:<20.12}{1:<20.12}{2:<20.12}".format(*vertex), file=export_handle)

            # Determine polygon list size
            size = 0
            for pi in panel_indices:
                size += len(pi)

            # Write panel polygons
            print("POLYGONS {0} {1}".format(len(panel_indices), size), file=export_handle)
            for panel in panel_indices:
                print(" ".join([str(i) for i in panel]), file=export_handle)


    def _get_vtk_data(self, **kwargs):
        # Assembles a list of vertices and panel vertex indices for creating a vtk mesh

        # Get vtk data
        raw_vertices = []
        N_raw_vert = 0
        panel_N = []
        for segment in self.segments:
            for panel in segment.get_vtk_panel_vertices(**kwargs):
                N_raw_vert += len(panel)
                panel_N.append(len(panel))
                raw_vertices.extend(panel)

        # Get vertex list
        raw_vertices = np.array(raw_vertices)
        vertices, inverse_indices = np.unique(raw_vertices, return_inverse=True, axis=0)
        panel_vertex_indices = []
        i = 0
        for N in panel_N:
            panel_vertex_indices.append([N, *inverse_indices[i:i+N]])
            i += N

        return vertices, panel_vertex_indices


    def export_stp(self, **kwargs):
        """Exports .STEP files representing the aircraft.

        Parameters
        ----------
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

        # Export wing segment parts
        for _,segment in self.wing_segments.items():
            segment.export_stp(airplane_name=self.name, **kwargs)


    def export_dxf(self, **kwargs):
        """Exports .dxf files representing the aircraft.

        Parameters
        ----------
        file_tag : str, optional
            Optional tag to prepend to output filename default. The output files will be named "<AIRCRAFT_NAME>_<WING_NAME>.stp".

        section_resolution : int, optional
            Number of points to use in discretizing the airfoil section outline. Defaults to 200.
        
        number_guide_curves : int, optional
            Number of guidecurves to create. Defaults to 2 (one at the leading edge, one at the trailing edge).
        
        export_english_units : bool, optional
            Whether to export the dxf file in English units. Defaults to True.

        dxf_line_type : str, optional
            Type of line to be used in the .dxf file creation. Options include 'line', 'spline', and 'polyline'. Defaults to 'spline'.
        
        export_as_prismoid : bool, optional
            Whether to export each airfoil as a rectangle. Forces number_guide_curves to 4 and section_resolution to 5. Defaults to False.
        """

        # Export wing segment parts
        for _,segment in self.wing_segments.items():
            segment.export_dxf(self.name, **kwargs)