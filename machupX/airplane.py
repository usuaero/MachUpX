from .helpers import import_value, quaternion_to_euler, euler_to_quaternion, check_filepath, quaternion_transform, quaternion_inverse_transform
from .wing_segment import WingSegment
from airfoil_db import Airfoil

import json
import numpy as np
import math as m
import scipy.integrate as integ
from stl import mesh
import sys
import os
import warnings
import copy

class Airplane:
    """A class defining an airplane.

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
        Vector giving the wind velocity in flat-earth coordinates at the 
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

        self.name = name
        self._unit_sys = unit_system
        self._scene = scene
        
        self.wing_segments = {}
        self._airfoil_database = {}
        self._N = 0

        self._load_params(airplane_input)
        self.set_state(**init_state, v_wind=v_wind)
        self._create_airfoil_database()
        self._create_origin_segment()
        self._load_wing_segments()
        self._check_reference_params()
        self._initialize_controls(init_control_state)


    def _load_params(self, airplane_input):
        if isinstance(airplane_input, str):
            # Load JSON object
            check_filepath(airplane_input, ".json")
            with open(airplane_input) as json_handle:
                self._input_dict = json.load(json_handle)
        elif isinstance(airplane_input, dict):
            self._input_dict = airplane_input
        else:
            raise IOError("{0} is not an allowed airplane definition. Must be path or dictionary.".format(airplane_input))

        # Set airplane global params
        self.CG = import_value("CG", self._input_dict, self._unit_sys, [0,0,0])
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
            Aircraft velocity.

        alpha : float, optional
            Aircraft angle of attack. Defaults to 0.0.

        beta : float, optional
            Aircraft sideslip angle. Defaults to 0.0.

        orientation : list, optional
            3-element vector given in the aircraft Euler angles or 4-element vector giving the
            aircraft orientation quaternion. Defaults to [0.0, 0.0, 0.0] or [1.0, 0.0, 0.0, 0.0].

        angular_rates : list, optional
            Body-fixed rotation rates. Defaults to [0.0, 0.0, 0.0].

        v_wind : list, optional
            Local wind vector. Defaults to [0.0, 0.0, 0.0].
        """

        # Get kwargs
        v_wind = kwargs.get("v_wind", [0.0, 0.0, 0.0])

        # Check for ussers using depreciated state definitions
        self.state_type = import_value("type", kwargs, self._unit_sys, "none")
        if self.state_type != "none":
            raise IOError("Use of 'state_type' is no longer supported. See documentation for details.")

        # Get position and angular rates
        self.p_bar = import_value("position", kwargs, self._unit_sys, [0.0, 0.0, 0.0])
        self.w = import_value("angular_rates", kwargs, self._unit_sys, [0.0, 0.0, 0.0])

        # Set up orientation quaternion
        self.q = import_value("orientation", kwargs, self._unit_sys, [1.0, 0.0, 0.0, 0.0]) # Default aligns the aircraft with the flat-earth coordinates

        if self.q.shape[0] == 3: # Euler angles
            self.q = euler_to_quaternion(np.radians(self.q))

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
            self.v = np.array([100.0, 0.0, 0.0]) # Keeps the following call to set_aerodynamic_state() from breaking
            self.set_aerodynamic_state(alpha=alpha, beta=beta, velocity=v_value, v_wind=v_wind)

        # Earth-fixed velocity vector
        elif isinstance(v_value, np.ndarray) and v_value.shape == (3,):

            # Make sure alpha and beta haven't also been given
            if "alpha" in list(kwargs.keys()) or "beta" in list(kwargs.keys()):
                raise IOError("Alpha and beta are not allowed when the freestream velocity is a vector.")

            # Store
            self.v = v_value

        else:
            raise IOError("{0} is not an allowable velocity definition.".format(v_value))


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
        v = quaternion_transform(self.q, self.v-v_wind)

        # Calculate values
        alpha = m.degrees(m.atan2(v[2], v[0]))
        beta = m.degrees(m.atan2(v[1], v[0]))
        velocity = np.linalg.norm(v)
        return alpha, beta, velocity


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

        # Calculate trigonometric values
        C_a = m.cos(m.radians(alpha))
        S_a = m.sin(m.radians(alpha))
        C_B = m.cos(m.radians(beta))
        S_B = m.sin(m.radians(beta))

        # Determine freestream velocity components in body-fixed frame (Mech of Flight Eqs. 7.1.10-12)
        v_inf_b = np.zeros(3)
        denom = 1/m.sqrt(1-S_a*S_a*S_B*S_B)
        v_inf_b[0] = velocity*C_a*C_B*denom
        v_inf_b[1] = velocity*C_a*S_B*denom
        v_inf_b[2] = velocity*S_a*C_B*denom

        # Transform to earth-fixed coordinates
        self.v = v_wind+quaternion_inverse_transform(self.q, v_inf_b)


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

        side = input_dict.get("side")
        if not (side == "left" or side == "right" or side == "both"):
            raise IOError("{0} is not a proper side designation.".format(side))

        if side == "left" or side == "both":
            self.wing_segments[wing_segment_name+"_left"] = self._origin_segment.attach_wing_segment(wing_segment_name+"_left", input_dict, "left", self._unit_sys, self._airfoil_database)
            self._N += self.wing_segments[wing_segment_name+"_left"].N

        if side == "right" or side == "both":
            self.wing_segments[wing_segment_name+"_right"] = self._origin_segment.attach_wing_segment(wing_segment_name+"_right", input_dict, "right", self._unit_sys, self._airfoil_database)
            self._N += self.wing_segments[wing_segment_name+"_right"].N

        if recalculate_geometry:
            self._calculate_geometry()


    def _load_wing_segments(self):
        # Reads in the wing segments from the input dict and attaches them
        for key in self._input_dict.get("wings", {}):
            self.add_wing_segment(key, self._input_dict["wings"][key], recalculate_geometry=False)

        # Perform geometry calculations
        self._calculate_geometry()


    def _calculate_geometry(self):
        # Figures out which wing segments are contiguous and sets up the lists of control points and vortex nodes

        # Initialize arrays
        self.c_bar = np.zeros(self._N) # Average chord
        self.dS = np.zeros(self._N) # Differential planform area
        self.PC = np.zeros((self._N,3)) # Control point location
        self.PC_span_locs = np.zeros(self._N) # Control point span locations
        self.P0 = np.zeros((self._N,self._N,3)) # Inbound vortex node location (effective locus of aerodynamic centers)
        self.P0_span__locs = np.zeros(self._N)
        self.P1 = np.zeros((self._N,self._N,3)) # Outbound vortex node location (effective locus of aerodynamic centers)
        self.P1_span__locs = np.zeros(self._N)
        self.P0_joint = np.zeros((self._N,self._N,3)) # Inbound vortex joint node location (effective locus of aerodynamic centers)
        self.P1_joint = np.zeros((self._N,self._N,3)) # Outbound vortex joint node location (effective locus of aerodynamic centers)
        self.u_a = np.zeros((self._N,3)) # Section unit vectors
        self.u_n = np.zeros((self._N,3))
        self.u_s = np.zeros((self._N,3))
        reid_corr = np.zeros(self._N) # Whether to use Reid corrections
        sigma_blend = np.zeros(self._N) # Blending distance
        delta_joint = np.zeros(self._N) # Joint length

        # Group wing segments into wings
        self._sort_segments_into_wings()

        # Gather segment data
        self._wing_N = []
        self._wing_slices = []
        self._wing_node_spans = []
        cur_index = 0
        for i in range(self._num_wings):

            # Reset params for this wing
            self._wing_N.append(0)
            cur_span_from_left_tip = 0.0
            wing_node_spans = []

            # Loop through segments
            for j,segment in enumerate(self._segments_in_wings[i]):

                # Determine slice for this segment
                N = segment.N
                self._wing_N[i] += N
                cur_slice = slice(cur_index, cur_index+N)

                # Store section geometry
                self.c_bar[cur_slice] = segment.c_bar_cp
                self.PC[cur_slice,:] = segment.control_points
                self.dS[cur_slice] = segment.dS

                # General NLL parameters
                reid_corr[cur_slice] = segment.reid_corr
                sigma_blend[cur_slice] = segment.sigma_blend
                delta_joint[cur_slice] = segment.delta_joint

                # Store control point span locations
                if segment.side == "left":
                    self.PC_span_locs[cur_slice] = cur_span_from_left_tip+(1.0-segment.cp_span_locs)*segment.b
                else:
                    self.PC_span_locs[cur_slice] = cur_span_from_left_tip+segment.cp_span_locs*segment.b
                
                # Store node span locations
                if segment.side == "left":
                    node_spans = cur_span_from_left_tip+(1.0-segment.node_span_locs)*segment.b
                else:
                    node_spans = cur_span_from_left_tip+segment.node_span_locs*segment.b
                if j == 0:
                    wing_node_spans.append(node_spans)
                else:
                    wing_node_spans.append(node_spans[1:])

                # Store original node locations, to be updated point by point
                self.P0[:,cur_slice,:] = segment.nodes[:-1]
                self.P1[:,cur_slice,:] = segment.nodes[1:]

                # Section direction vectors
                self.u_a[cur_slice,:] = segment.u_a_cp
                self.u_n[cur_slice,:] = segment.u_n_cp
                self.u_s[cur_slice,:] = segment.u_s_cp

                # Update for next segment
                cur_index += N
                cur_span_from_left_tip += segment.b

            # Determine slice for this wing
            self._wing_slices.append(slice(cur_index-self._wing_N[i], cur_index))

            # Concatenate nodes
            self._wing_node_spans.append(np.concatenate(wing_node_spans))

        # Calculate effective loci of aerodynamic centers
        cur_wing = 0
        gradient = np.gradient(self.PC[self._wing_slices[cur_wing]], self.PC_span_locs[self._wing_slices[cur_wing]], edge_order=2, axis=0)
        for i in range(self._N):

            # Check if we've moved beyond the correct wing
            if i >= self._wing_slices[cur_wing].stop:
                cur_wing += 1

                # Update gradient
                gradient = np.gradient(self.PC[self._wing_slices[cur_wing]], self.PC_span_locs[self._wing_slices[cur_wing]], edge_order=2, axis=0)

            # General NLL corrections; if this is skipped, the node locations remain unchanged
            if reid_corr[i]:

                # Get control point of interest
                wing_index = i-self._wing_slices[cur_wing].start # Index of control point within this wing
                PC = self.PC[i,:]
                PC_span = self.PC_span_locs[i]
                PC_deriv = gradient[wing_index]

                # Create straight locus of AC based off the current the control point
                ds = self._wing_node_spans[cur_wing]-PC_span
                straight_ac = PC + PC_deriv*ds

                # Get interpolation function
                blend = np.exp(-sigma_blend[i]*ds*ds)

        # Place vortex joint locations

        #    # Get normal vectors to the AC locus lying in the plane of the chord
        #    # These equations ensure the joint vector is orthogonal to the ac tangent and lies in the same plane as the
        #    # ac tangent and the axial vector (i.e. chord line). You get these by solving
        #    #
        #    #   < u_j, T > = 0
        #    #   < u_j, u_a > > 0
        #    #   u_j = c1*u_a+c2*T
        #    #
        #    T = np.gradient(self.nodes, self._node_span_locs*self.b, edge_order=2, axis=0)
        #    T = T/np.linalg.norm(T, axis=1)[:,np.newaxis]
        #    u_a = self._get_axial_vec(self._node_span_locs)
        #    k = np.einsum('ij,ij->i', T, u_a)
        #    c1 = np.sqrt(1/(1-k*k))
        #    c2 = -c1*k
        #    u_j = c1[:,np.newaxis]*u_a+c2[:,np.newaxis]*T
        #    u_j = u_j/np.linalg.norm(u_j)

        #    # Get offset
        #    c = self.get_chord(self._node_span_locs)
        #    self.nodes_prime = self.nodes+c[:,np.newaxis]*self._delta_joint*u_j


    def _sort_segments_into_wings(self):
        # Groups segments into wings for calculating effective ac loci
        wing_IDs = []

        # Get user selections
        for segment in self.wing_segments.values():
            if segment.wing_ID is not None and segment.wing_ID not in wing_IDs:
                wing_IDs.append(segment.wing_ID)

        # To allow for max() check in while loop
        if len(wing_IDs) == 0:
            wing_IDs.append(-1)

        # Assign IDs and group
        curr_ID = 0
        self._segments_in_wings = []
        finished = False
        while not finished or curr_ID <= max(wing_IDs):

            # Create storage
            self._segments_in_wings.append([])

            # Check if the user has called for the current ID
            if curr_ID in wing_IDs:

                # Gather segments with that ID
                for segment in self.wing_segments.values():
                    if segment.wing_ID == curr_ID:
                        self._segments_in_wings[curr_ID].append(segment)

            else:
                
                # Assign current ID to segment with no specified wing
                for segment in self.wing_segments.values():

                    # Look for segment with no ID
                    if segment.wing_ID is None:
                        segment.wing_ID = curr_ID
                        self._segments_in_wings[curr_ID].append(segment)

                        # Look for matching half
                        segment_root_name = segment.name.replace("_right", "").replace("_left", "")
                        for partner_segment in self.wing_segments.values():
                            if segment_root_name in partner_segment.name and partner_segment.wing_ID is None:
                                partner_segment.wing_ID = curr_ID
                                self._segments_in_wings[curr_ID].append(partner_segment)
                                break

                        break
                
                # If there are no unspecified wings left
                else:
                    finished = True

            curr_ID += 1

        # Store number of wings
        self._num_wings = curr_ID

        # Delete empty wings
        empty = []
        for i in range(self._num_wings):
            if len(self._segments_in_wings[i]) == 0:
                empty.append(i)
                self._num_wings -= 1
        for index in empty[::-1]:
            del self._segments_in_wings[index]

        # Reassign IDs
        for i in range(self._num_wings):
            for segment in self._segments_in_wings[i]:
                segment.wing_ID = i

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
                if wing_segment.is_main and wing_segment._side == "right":
                    self.l_ref_lat += wing_segment.b

        # Longitudinal reference length
        if self.l_ref_lon == -1:
            self.l_ref_lon = self.S_w/(2*self.l_ref_lat)


    def delete_wing_segment(self, wing_segment_name):
        """Removes the specified wing segment from the airplane. Removes both sides.

        Parameters
        ----------
        wing_segment_name : str
            Name of the wing segment.

        Returns
        -------

        Raises
        ------
        ValueError
            If the wing segment does not exist

        RuntimeError
            If the wing segment has other segments attached to it.
        """
        #TODO: Do this
        pass


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


    def get_num_cps(self):
        """Returns the total number of control points on the aircraft.

        Returns
        -------
        int
            Number of control points on the aircraft.
        """
        return self._N


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
        """

        # Check for .stl file
        filename = kwargs.get("filename")
        if ".stl" not in filename:
            raise IOError("{0} is not a .stl file.".format(filename))

        # Loop through segments
        num_facets = 0
        vector_dict = {}
        section_resolution = kwargs.get("section_resolution", 200)
        for segment_name, segment_object in self.wing_segments.items():
            vectors = segment_object.get_stl_vectors(section_resolution)
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
        """

        # Export wing segment parts
        for _,segment in self.wing_segments.items():
            segment.export_dxf(self.name, **kwargs)