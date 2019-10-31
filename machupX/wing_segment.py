from .helpers import *

import json
import numpy as np
import scipy.integrate as integ
import scipy.interpolate as interp


class WingSegment:
    """A class defining a segment of a lifting surface.

    Parameters
    ----------
    name : string
        Name of the wing segment.

    input_dict : dict
        Dictionary describing the geometry of the segment.

    side : string
        The side the wing segment is added on, either "right" or "left".

    unit_sys : str
        Default system of units.

    airfoil_dict : dict
        Dictionary of airfoil objects. Must contain the airfoils specified for this wing segment.

    origin : vector
        Origin (root) coordinates of the wing segment in body-fixed coordinates.

    Returns
    -------
    WingSegment
        Returns a newly created WingSegment object.

    Raises
    ------
    IOError
        If the input is improperly specified.
    """

    def __init__(self, name, input_dict, side, unit_sys, airfoil_dict, origin=[0.0, 0.0, 0.0]):

        self.name = name
        self._input_dict = input_dict
        self._unit_sys = unit_sys
        self._side = side
        self._origin = np.asarray(origin)

        self._attached_segments = {}
        self._getter_data = {}
        
        self.ID = self._input_dict.get("ID")
        if self.ID == 0 and name != "origin":
            raise IOError("Wing segment ID for {0} may not be 0.".format(name))

        if self.ID != 0: # These do not need to be run for the origin segment
            self._initialize_params()
            self._initialize_getters()
            self._initialize_airfoils(airfoil_dict)
            self._setup_control_surface(self._input_dict.get("control_surface", None))

            # These make repeated calls for geometry information faster. Should be called again if geometry changes.
            self._setup_cp_data()
            self._setup_node_data()

    
    def _initialize_params(self):

        # Set global params
        self.is_main = self._input_dict.get("is_main", None)
        self.b = import_value("semispan", self._input_dict, self._unit_sys, None)

        # Grid parameters
        grid_dict = self._input_dict.get("grid", {})
        self._N = grid_dict.get("N", 40)
        distribution = grid_dict.get("distribution", "cosine_cluster")
        flap_edge_cluster = grid_dict.get("flap_edge_cluster", True)
        extra_discont = grid_dict.get("cluster_points", [])

        # Set origin offset
        self._delta_origin = np.zeros(3)
        connect_dict = self._input_dict.get("connect_to", {})
        self._delta_origin[0] = connect_dict.get("dx", 0.0)
        self._delta_origin[1] = connect_dict.get("dy", 0.0)
        self._delta_origin[2] = connect_dict.get("dz", 0.0)

        if self._side == "left":
            self._delta_origin[1] -= connect_dict.get("y_offset", 0.0)

        else:
            self._delta_origin[1] += connect_dict.get("y_offset", 0.0)

        # Create arrays of span locations used to generate nodes and control points
        if distribution == "cosine_cluster": # Cosine clustering

            discont = []

            # Add flap edges
            if flap_edge_cluster:
                flap_dict = self._input_dict.get("control_surface", None)
                if flap_dict is not None:
                    discont.append(flap_dict.get("root_span", 0.0))
                    discont.append(flap_dict.get("tip_span", 1.0))

            # Add user-specified discontinuities
            for discont_span_frac in extra_discont:
                discont.append(discont_span_frac)

            # Ignore discontinuities at wingtip
            while True:
                try:
                    discont.remove(1.0)
                except ValueError:
                    break

            # Ignore discontinuities at wing root
            while True:
                try:
                    discont.remove(0.0)
                except ValueError:
                    break

            # Sort discontinuities
            discont.sort()
            discont.append(1.0) # I know this is kinda redundant, but it's the best thing I could think of
            discont.insert(0, 0.0)

            # Determine number of sections and number of control points in each section
            num_sec = len(discont)-1
            sec_N = []
            for i in range(num_sec):
                N = round(self._N*(discont[i+1]-discont[i]))
                sec_N.append(N)

            # Check all the points are accounted for
            diff = sum(sec_N)-self._N
            if diff != 0:
                sec_N[0] -= diff # Use the root segment to make up the difference

            # Create node and control point span locations
            node_span_locs = [0.0]
            cp_span_locs = []

            for i in range(num_sec):

                node_theta_space = list(np.linspace(0.0, np.pi, sec_N[i]+1))
                for theta in node_theta_space[1:]:
                    s = 0.5*(1-np.cos(theta)) # Span fraction
                    node_span_locs.append(discont[i]+s*(discont[i+1]-discont[i]))

                cp_theta_space = np.linspace(np.pi/sec_N[i], np.pi, sec_N[i])-np.pi/(2*sec_N[i])
                for theta in cp_theta_space:
                    s = 0.5*(1-np.cos(theta)) # Span fraction
                    cp_span_locs.append(discont[i]+s*(discont[i+1]-discont[i]))

            self._node_span_locs = np.array(node_span_locs)
            self._cp_span_locs = np.array(cp_span_locs)

        elif distribution == "linear": # Linear spacing
            self._node_span_locs = np.linspace(0.0, 1.0, self._N+1)
            self._cp_span_locs = np.linspace(1/(2*self._N), 1.0-1/(2*self._N), self._N)

        elif isinstance(distribution, list): # User-specified distribution
            if len(distribution) != self._N*2+1:
                raise IOError("User specified distribution must have length of 2*N+1. Got length {0}, needed length {1}.".format(len(distribution), self._N*2+1))

            self._node_span_locs = np.array(distribution[0::2])
            self._cp_span_locs = np.array(distribution[1::2])

        else:
            raise IOError("Distribution type {0} not recognized for wing segment {1}.".format(distribution, self.name))

        # In order to follow the airfoil sign convention (i.e. positive vorticity creates positive lift) 
        # node and control point locations must always proceed from left to right.
        if self._side == "left":
            self._node_span_locs = self._node_span_locs[::-1]
            self._cp_span_locs = self._cp_span_locs[::-1]


    def _initialize_getters(self):
        # Sets getters for functions which are a function of span

        # Twist
        twist_data = import_value("twist", self._input_dict, self._unit_sys, 0)
        self.get_twist = self._build_getter_linear_f_of_span(twist_data, "twist", angular_data=True)

        # Dihedral
        dihedral_data = import_value("dihedral", self._input_dict, self._unit_sys, 0)
        self.get_dihedral = self._build_getter_linear_f_of_span(dihedral_data, "dihedral", angular_data=True)

        # Sweep
        sweep_data = import_value("sweep", self._input_dict, self._unit_sys, 0)
        self.get_sweep = self._build_getter_linear_f_of_span(sweep_data, "sweep", angular_data=True)

        # Chord
        chord_data = import_value("chord", self._input_dict, self._unit_sys, 1.0)
        if isinstance(chord_data, tuple): # Elliptic distribution
            self.get_chord = self._build_elliptic_chord_dist(chord_data[1])
        else: # Linear distribution
            self.get_chord = self._build_getter_linear_f_of_span(chord_data, "chord")

        # Aerodynamic center offset
        ac_offset_data = import_value("ac_offset", self._input_dict, self._unit_sys, 0)
        self._get_ac_offset = self._build_getter_linear_f_of_span(ac_offset_data, "ac_offset")


    def _build_getter_linear_f_of_span(self, data, name, angular_data=False):
        # Defines a getter function for data which is a function of span

        if isinstance(data, float): # Constant
            if angular_data:
                self._getter_data[name] = np.radians(data).item()
            else:
                self._getter_data[name] = data

            def getter(span):
                converted = False
                if isinstance(span, float):
                    converted = True
                    span = np.asarray(span)[np.newaxis]

                data = np.full(span.shape, self._getter_data[name])
                if converted:
                    span = span.item()

                return data
        
        else: # Array
            self._getter_data[name] = data

            def getter(span):
                converted = False
                if isinstance(span, float):
                    converted = True
                    span = np.asarray(span)[np.newaxis]

                if angular_data:
                    data = np.interp(span, self._getter_data[name][:,0], np.radians(self._getter_data[name][:,1]))
                else:
                    data = np.interp(span, self._getter_data[name][:,0], self._getter_data[name][:,1])
                if converted:
                    span = span.item()

                return data

        return getter


    def _build_elliptic_chord_dist(self, root_chord):
        # Creates a getter which will return the chord length as a function of span fraction according to an elliptic distribution
        self._root_chord = root_chord

        def getter(span_frac):
            return self._root_chord*np.sqrt(1-span_frac**2)

        return getter


    def _initialize_airfoils(self, airfoil_dict):
        # Picks out the airfoils used in this wing segment and stores them. Also 
        # initializes airfoil coefficient getters

        # Get which airfoils are specified for this segment
        default_airfoil = list(airfoil_dict.keys())[0]
        airfoil = import_value("airfoil", self._input_dict, self._unit_sys, default_airfoil)

        self._airfoils = []
        self._airfoil_spans = []
        self._num_airfoils = 0

        # Setup data table
        if isinstance(airfoil, str): # Constant airfoil

            if not airfoil in list(airfoil_dict.keys()):
                raise IOError("'{0}' must be specified in 'airfoils'.".format(airfoil))

            # Just put the same airfoil at the root and the tip
            self._airfoils.append(airfoil_dict[airfoil])
            self._airfoils.append(airfoil_dict[airfoil])
            self._airfoil_spans.append(0.0)
            self._airfoil_spans.append(1.0)
            self._num_airfoils = 2


        elif isinstance(airfoil, np.ndarray): # Distribution of airfoils
            self._airfoil_data = np.empty((airfoil.shape[0], airfoil.shape[1]+1), dtype=None)

            # Store each airfoil and its span location
            for row in airfoil:

                name = row[1].item()

                try:
                    self._airfoils.append(airfoil_dict[name])
                except NameError:
                    raise IOError("'{0}' must be specified in 'airfoils'.".format(name))

                self._airfoil_spans.append(float(row[0]))
                self._num_airfoils += 1

        else:
            raise IOError("Airfoil definition must a be a string or an array.")

        self._airfoil_spans = np.asarray(self._airfoil_spans)


    def _setup_control_surface(self, control_dict):
        # Sets up the control surface on this wing segment

        # These values are needed whether or not a control surface exists
        self._delta_flap = 0.0 # Positive deflection is down
        self._has_control_surface = False
        self._Cm_delta_flap = 0.0

        if control_dict is not None:
            self._has_control_surface = True

            self._cp_flap_chord_frac = np.zeros(self._N)
            self._control_mixing = {}

            # Determine which control points are affected by the control surface
            root_span = control_dict.get("root_span", 0.0)
            tip_span = control_dict.get("tip_span", 1.0)
            self._cp_in_control_surface = (self._cp_span_locs >= root_span) & (self._cp_span_locs <= tip_span)

            # Determine the flap chord fractions at each control point
            chord_data = import_value("chord_fraction", control_dict, self._unit_sys, 0.25)
            if isinstance(chord_data, float): # Constant chord fraction
                self._cp_flap_chord_frac[self._cp_in_control_surface] = chord_data
            else: # Variable chord fraction
                if chord_data[0,0] != root_span or chord_data[-1,0] != tip_span:
                    raise IOError("Endpoints of flap chord distribution must match specified root and tip span locations.")
                self._cp_flap_chord_frac[self._cp_in_control_surface] = np.interp(self._cp_span_locs[self._cp_in_control_surface], chord_data[:,0], chord_data[:,1])

            # Store mixing
            self._control_mixing = control_dict.get("control_mixing", {})
            is_sealed = control_dict.get("is_sealed", True)

            # Determine flap efficiency for altering angle of attack
            theta_f = np.arccos(2*self._cp_flap_chord_frac-1)
            eps_flap_ideal = 1-(theta_f-np.sin(theta_f))/np.pi

            # Based off of Mechanics of Flight Fig. 1.7.4
            hinge_eff = 3.9598*np.arctan((self._cp_flap_chord_frac+0.006527)*89.2574+4.898015)-5.18786
            if not is_sealed:
                hinge_eff *= 0.8

            self._eta_h_esp_f = eps_flap_ideal*hinge_eff

            # Determine flap efficiency for changing moment coef
            self._Cm_delta_flap = (np.sin(2*theta_f)-2*np.sin(theta_f))/4


    def _setup_cp_data(self):
        # Creates and stores vectors of important data at each control point
        self.u_a_cp = self._get_axial_vec(self._cp_span_locs)
        self.u_n_cp = self._get_normal_vec(self._cp_span_locs)
        self.u_s_cp = self._get_span_vec(self._cp_span_locs)
        self.control_points = self._get_section_ac_loc(self._cp_span_locs)
        self.c_bar_cp = self._get_cp_avg_chord_lengths()
        self.dS = abs(self._node_span_locs[1:]-self._node_span_locs[:-1])*self.b*self.c_bar_cp
        self.twist_cp = self.get_twist(self._cp_span_locs)
        self.dihedral_cp = self.get_dihedral(self._cp_span_locs)
        self.sweep_cp = self.get_sweep(self._cp_span_locs)


    def _setup_node_data(self):
        # Creates and stores vectors of important data at each node
        self.nodes = self._get_section_ac_loc(self._node_span_locs)


    def attach_wing_segment(self, wing_segment_name, input_dict, side, unit_sys, airfoil_dict):
        """Attaches a wing segment to the current segment or one of its children.
        
        Parameters
        ----------
        wing_segment_name : str
            Name of the wing segment to attach.

        input_dict : dict
            Dictionary describing the wing segment to attach.

        side : str
            Which side this wing segment goes on. Can only be "left" or "right"

        unit_sys : str
            The unit system being used. "English" or "SI".

        airfoil_dict : dict
            Dictionary of airfoil objects the wing segment uses to initialize its own airfoils.

        Returns
        -------
        WingSegment
            Returns a newly created wing segment.

        Raises
        ------
        RuntimeError
            If the segment could not be added.

        """

        # This can only be called on the origin segment
        if self.ID != 0:
            raise RuntimeError("Please add segments only at the origin segment.")

        else:
            return self._attach_wing_segment(wing_segment_name, input_dict, side, unit_sys, airfoil_dict)


    def _attach_wing_segment(self, wing_segment_name, input_dict, side, unit_sys, airfoil_dict):
        # Recursive function for attaching a wing segment.

        parent_ID = input_dict.get("connect_to", {}).get("ID", 0)
        if self.ID == parent_ID: # The new segment is supposed to attach to this one

            # Determine the connection point
            if input_dict.get("connect_to", {}).get("location", "tip") == "root":
                attachment_point = self.get_root_loc()
            else:
                attachment_point = self.get_tip_loc()

            self._attached_segments[wing_segment_name] = WingSegment(wing_segment_name, input_dict, side, unit_sys, airfoil_dict, attachment_point)

            return self._attached_segments[wing_segment_name] # Return reference to newly created wing segment

        else: # We need to recurse deeper
            result = False
            for key in self._attached_segments:
                if side not in key: # A right segment only ever attaches to a right segment and same with left
                    continue
                result = self._attached_segments[key]._attach_wing_segment(wing_segment_name, input_dict, side, unit_sys, airfoil_dict)
                if result is not False:
                    break

            if self.ID == 0 and not result:
                raise RuntimeError("Could not attach wing segment {0}. Check ID of parent is valid.".format(wing_segment_name))

            return result


    def _get_attached_wing_segment(self, wing_segment_name):
        # Returns a reference to the specified wing segment.
        try:
            # See if it is attached to this wing segment
            return self._attached_segments[wing_segment_name]
        except KeyError:
            # Otherwise
            result = False
            for key in self._attached_segments:
                result = self._attached_segments[key]._get_attached_wing_segment(wing_segment_name)
                if result:
                    break

            return result


    def get_root_loc(self):
        """Returns the location of the root quarter-chord.

        Returns
        -------
        ndarray
            Location of the root quarter-chord.
        """
        if self.ID == 0:
            return self._origin
        else:
            return self._origin+self._delta_origin


    def get_tip_loc(self):
        """Returns the location of the tip quarter-chord.

        Returns
        -------
        ndarray
            Location of the tip quarter-chord.
        """
        if self.ID == 0:
            return self._origin
        else:
            return self._get_quarter_chord_loc(1.0)


    def _get_quarter_chord_loc(self, span):
        #Returns the location of the quarter-chord at the given span fraction.
        if isinstance(span, float):
            converted = True
            span_array = np.asarray(span)[np.newaxis]
        else:
            converted = False
            span_array = np.asarray(span)

        # Integrate sweep and dihedral along the span to get the location
        ds = np.zeros((span_array.shape[0],3))
        for i, span in enumerate(span_array):
            ds[i,0] = integ.quad(lambda s : -np.tan(self.get_sweep(s)), 0, span)[0]*self.b
            if self._side == "left":
                ds[i,1] = integ.quad(lambda s : -np.cos(self.get_dihedral(s)), 0, span)[0]*self.b
            else:
                ds[i,1] = integ.quad(lambda s : np.cos(self.get_dihedral(s)), 0, span)[0]*self.b
            ds[i,2] = integ.quad(lambda s : -np.sin(self.get_dihedral(s)), 0, span)[0]*self.b

        qc_loc = self.get_root_loc()+ds
        if converted:
            qc_loc = qc_loc.flatten()

        return qc_loc


    def _get_axial_vec(self, span):
        # Returns the axial vector at the given span locations
        if isinstance(span, float):
            span_array = np.asarray(span)[np.newaxis]
        else:
            span_array = np.asarray(span)

        twist = self.get_twist(span_array)
        dihedral = self.get_dihedral(span_array)
        
        C_twist = np.cos(twist)
        S_twist = np.sin(twist)
        C_dihedral = np.cos(dihedral)
        S_dihedral = np.sin(dihedral)

        if self._side == "left":
            return np.asarray([-C_twist, -S_twist*S_dihedral, S_twist*C_dihedral]).T
        else:
            return np.asarray([-C_twist, S_twist*S_dihedral, S_twist*C_dihedral]).T


    def _get_normal_vec(self, span):
        # Returns the normal vector at the given span locations
        if isinstance(span, float):
            span_array = np.asarray(span)[np.newaxis]
        else:
            span_array = np.asarray(span)

        twist = self.get_twist(span_array)
        dihedral = self.get_dihedral(span_array)
        
        C_twist = np.cos(twist)
        S_twist = np.sin(twist)
        C_dihedral = np.cos(dihedral)
        S_dihedral = np.sin(dihedral)

        if self._side == "left":
            return np.asarray([-S_twist, C_twist*S_dihedral, -C_twist*C_dihedral]).T
        else:
            return np.asarray([-S_twist, -C_twist*S_dihedral, -C_twist*C_dihedral]).T


    def _get_span_vec(self, span):
        # Returns the normal vector at the given span locations
        if isinstance(span, float):
            span_array = np.asarray(span)[np.newaxis]
        else:
            span_array = np.asarray(span)

        dihedral = self.get_dihedral(span_array)

        C_dihedral = np.cos(dihedral)
        S_dihedral = np.sin(dihedral)

        if self._side == "left":
            return np.asarray([np.zeros(span_array.size), -C_dihedral, -S_dihedral]).T
        else:
            return np.asarray([np.zeros(span_array.size), -C_dihedral, S_dihedral]).T


    def _get_section_ac_loc(self, span):
        #Returns the location of the section aerodynamic center at the given span fraction.
        loc = self._get_quarter_chord_loc(span)
        loc += (self._get_ac_offset(span)*self.get_chord(span))[:,np.newaxis]*self._get_axial_vec(span)
        return loc


    def _get_cp_avg_chord_lengths(self):
        #Returns the average local chord length at each control point on the segment.
        node_chords = self.get_chord(self._node_span_locs)
        return (node_chords[1:]+node_chords[:-1])/2


    def _airfoil_interpolator(self, interp_spans, sample_spans, coefs):
        # Interpolates the airfoil coefficients at the given span locations.
        # Allows for the coefficients having been evaluated as a function of 
        # span as well.
        # Solution found on stackoverflow
        i = np.arange(interp_spans.size)
        j = np.searchsorted(sample_spans, interp_spans) - 1
        j[np.where(j<0)] = 0 # Not allowed to go outside the array
        d = (interp_spans-sample_spans[j])/(sample_spans[j+1]-sample_spans[j])
        return (1-d)*coefs[i,j]+d*coefs[i,j+1]


    def get_cp_CLa(self, params):
        """Returns the lift slope at each control point.

        Parameters
        ----------
        params : ndarray
            Airfoil parameters.

        Returns
        -------
        float
            Lift slope
        """
        if params.shape[0] != self._N:
            raise ValueError("params with shape {0} does not match {1} control points.".format(params.shape, self._N))

        new_params = np.zeros((self._N,5))
        new_params[:,:3] = params
        if self._has_control_surface:
            new_params[:,3] = self._flap_eff
            new_params[:,4] = self._delta_flap

        CLas = np.zeros((self._N,self._num_airfoils))
        for i in range(self._N):
            for j in range(self._num_airfoils):
                CLas[i,j] = self._airfoils[j].get_CLa(new_params[i,:])

        return self._airfoil_interpolator(self._cp_span_locs, self._airfoil_spans, CLas)


    def get_cp_aL0(self, params):
        """Returns the zero-lift angle of attack at each control point. Used for the linear 
        solution to NLL.

        Parameters
        ----------
        params : ndarray
            Airfoil parameters.

        Returns
        -------
        float
            Zero lift angle of attack
        """
        if params.shape[0] != self._N:
            raise ValueError("params with shape {0} does not match {1} control points.".format(params.shape, self._N))

        new_params = np.zeros((self._N,5))
        new_params[:,:3] = params
        if self._has_control_surface:
            new_params[:,3] = self._flap_eff
            new_params[:,4] = self._delta_flap

        aL0s = np.zeros((self._N,self._num_airfoils))
        for i in range(self._N):
            for j in range(self._num_airfoils):
                aL0s[i,j] = self._airfoils[j].get_aL0(new_params[i,:])

        return self._airfoil_interpolator(self._cp_span_locs, self._airfoil_spans, aL0s)


    def get_cp_CLRe(self, params):
        """Returns the derivative of the lift coefficient with respect to Reynolds number at each control point

        Parameters
        ----------
        params : ndarray
            Airfoil parameters.

        Returns
        -------
        float
            Z
        """
        if params.shape[0] != self._N:
            raise ValueError("params with shape {0} does not match {1} control points.".format(params.shape, self._N))

        new_params = np.zeros((self._N,5))
        new_params[:,:3] = params
        if self._has_control_surface:
            new_params[:,3] = self._flap_eff
            new_params[:,4] = self._delta_flap

        CLRes = np.zeros((self._N,self._num_airfoils))
        for i in range(self._N):
            for j in range(self._num_airfoils):
                CLRes[i,j] = self._airfoils[j].get_CLRe(new_params[i,:])

        return self._airfoil_interpolator(self._cp_span_locs, self._airfoil_spans, CLRes)

    
    def get_cp_CLM(self, params):
        """Returns the derivative of the lift coefficient with respect to Mach number at each control point

        Parameters
        ----------
        params : ndarray
            Airfoil parameters.

        Returns
        -------
        float
            Z
        """
        if params.shape[0] != self._N:
            raise ValueError("params with shape {0} does not match {1} control points.".format(params.shape, self._N))

        new_params = np.zeros((self._N,5))
        new_params[:,:3] = params
        if self._has_control_surface:
            new_params[:,3] = self._flap_eff
            new_params[:,4] = self._delta_flap

        CLMs = np.zeros((self._N,self._num_airfoils))
        for i in range(self._N):
            for j in range(self._num_airfoils):
                CLMs[i,j] = self._airfoils[j].get_CLM(new_params[i,:])

        return self._airfoil_interpolator(self._cp_span_locs, self._airfoil_spans, CLMs)


    def get_cp_CL(self, params):
        """Returns the coefficient of lift at each control point as a function of params.

        Parameters
        ----------
        params : ndarray
            Airfoil parameters.

        Returns
        -------
        float or ndarray
            Coefficient of lift
        """
        if params.shape[0] != self._N:
            raise ValueError("params with shape {0} does not match {1} control points.".format(params.shape, self._N))

        new_params = np.zeros((self._N,5))
        new_params[:,:3] = params
        if self._has_control_surface:
            new_params[:,3] = self._flap_eff
            new_params[:,4] = self._delta_flap

        CLs = np.zeros((self._N,self._num_airfoils))
        for i in range(self._N):
            for j in range(self._num_airfoils):
                CLs[i,j] = self._airfoils[j].get_CL(new_params[i,:])

        return self._airfoil_interpolator(self._cp_span_locs, self._airfoil_spans, CLs)


    def get_cp_CD(self, params):
        """Returns the coefficient of drag at each control point as a function of params.

        Parameters
        ----------
        params : floats
            Airfoil parameters.

        Returns
        -------
        float
            Coefficient of drag
        """
        if params.shape[0] != self._N:
            raise ValueError("params with shape {0} does not match {1} control points.".format(params.shape, self._N))

        new_params = np.zeros((self._N,5))
        new_params[:,:3] = params
        if self._has_control_surface:
            new_params[:,3] = self._flap_eff
            new_params[:,4] = self._delta_flap

        CDs = np.zeros((self._N,self._num_airfoils))
        for i in range(self._N):
            for j in range(self._num_airfoils):
                CDs[i,j] = self._airfoils[j].get_CD(new_params[i,:])

        return self._airfoil_interpolator(self._cp_span_locs, self._airfoil_spans, CDs)


    def get_cp_Cm(self, params):
        """Returns the moment coefficient at each control point as a function of params.

        Parameters
        ----------
        params : floats
            Airfoil parameters.

        Returns
        -------
        float
            Moment coefficient
        """
        if params.shape[0] != self._N:
            raise ValueError("params with shape {0} does not match {1} control points.".format(params.shape, self._N))

        new_params = np.zeros((self._N,5))
        new_params[:,:3] = params
        if self._has_control_surface:
            new_params[:,3] = self._Cm_delta_flap
            new_params[:,4] = self._delta_flap

        Cms = np.zeros((self._N,self._num_airfoils))
        for i in range(self._N):
            for j in range(self._num_airfoils):
                Cms[i,j] = self._airfoils[j].get_Cm(new_params[i,:])

        return self._airfoil_interpolator(self._cp_span_locs, self._airfoil_spans, Cms)


    def get_outline_points(self):
        """Returns a set of points that represents the outline of the wing segment.
        
        Returns
        -------
        ndarray
            Array of outline points.
        """
        spans = np.linspace(0, 1, self._N)
        qc_points = self._get_quarter_chord_loc(spans)
        chords = self.get_chord(spans)
        axial_vecs = self._get_axial_vec(spans)

        points = np.zeros((self._N*2+1,3))

        # Leading edge
        points[:self._N,:] = qc_points - 0.25*(axial_vecs*chords[:,np.newaxis])

        # Trailing edge
        points[-2:self._N-1:-1,:] = qc_points + 0.75*(axial_vecs*chords[:,np.newaxis])

        # Complete the circle
        points[-1,:] = points[0,:]

        #TODO: Add control surface

        return points


    def apply_control(self, control_state, control_symmetry):
        """Applies the control deflection in degrees to this wing segment's control surface deflection.

        Parameters
        ----------
        control_state : dict
            A set of key-value pairs where the key is the name of the control and the 
            value is the deflection. For positive mapping values, a positive deflection 
            here will cause a downward deflection of symmetric control surfaces and 
            downward deflection of the right surface for anti-symmetric control surfaces.
            Units may be specified as in the input file. Any deflections not given will 
            default to zero.

        control_symmetry : dict
            Specifies which of the controls are symmetric
        """
        if not self._has_control_surface:
            return # Don't even bother...

        # Determine flap deflection
        self._delta_flap = 0.0
        for key in self._control_mixing:
            deflection = import_value(key, control_state, self._unit_sys, 0.0)
            if self._side == "right" or control_symmetry[key]:
                self._delta_flap += deflection*self._control_mixing.get(key, 0.0)
            else:
                self._delta_flap -= deflection*self._control_mixing.get(key, 0.0)

        # Determine flap efficiency
        # From a fit of Mechanics of Flight Fig. 1.7.5
        if self._delta_flap < 11:
            self._eta_defl = 1.0
        else:
            self._eta_defl = -8.71794871794872E-03*self._delta_flap+1.09589743589744
        self._flap_eff = self._eta_h_esp_f*self._eta_defl

        # Convert to radians
        self._delta_flap = np.radians(self._delta_flap)


    def get_cp_flap(self):
        """Returns the effective change in angle of attack due to the flap deflection.
        Used for the linear solution to NLL.
        """
        if self._has_control_surface:
            return self._flap_eff*self._delta_flap
        else:
            return 0.0


    def get_stl_vectors(self, section_res=200):
        """Calculates and returns the outline vectors required for 
        generating an .stl model of the wing segment.

        Parameters
        ----------
        section_res : int, optional
            Number of points to use in distcretizing the airfoil sections. Defaults to 200.

        Returns
        -------
        ndarray
            Array of outline vectors. First index is the facet index, second is the point
            index, third is the vector components.
        """

        # Collect airfoil outlines
        airfoil_outlines = {}
        for airfoil in self._airfoils:
            airfoil_outlines[airfoil.name] = airfoil.get_outline_points(section_res)

        # Discretize by node locations
        num_facets = self._N*(section_res-1)*2
        vectors = np.zeros((num_facets*3,3))

        # Generate vectors
        for i in range(self._N):

            # Root-ward node
            root_span = self._node_span_locs[i]
            root_outline = self._get_airfoil_outline_coords_at_span(root_span, section_res)

            # Tip-ward node
            tip_span = self._node_span_locs[i+1]
            tip_outline = self._get_airfoil_outline_coords_at_span(tip_span, section_res)

            # Create facets between the outlines
            for j in range(section_res-1):
                index = (2*i*(section_res-1)+2*j)*3

                vectors[index] = root_outline[j]
                vectors[index+1] = tip_outline[j+1]
                vectors[index+2] = tip_outline[j]

                vectors[index+3] = tip_outline[j+1]
                vectors[index+4] = root_outline[j]
                vectors[index+5] = root_outline[j+1]

        return vectors


    def _get_airfoil_outline_coords_at_span(self, span, N):
        # Returns the airfoil section outline in body-fixed coordinates at the specified span fraction with the specified number of points

        # Collect airfoil outlines
        airfoil_outlines = {}
        for airfoil in self._airfoils:
            airfoil_outlines[airfoil.name] = airfoil.get_outline_points(N)

        # Linearly interpolate outlines, ignoring twist, etc for now
        index = 0
        while True:
            if span >= self._airfoil_spans[index] and span <= self._airfoil_spans[index+1]:
                total_span = self._airfoil_spans[index+1]-self._airfoil_spans[index]
                root_weight = 1-abs(span-self._airfoil_spans[index])/total_span
                tip_weight = 1-abs(span-self._airfoil_spans[index+1])/total_span
                points = root_weight*airfoil_outlines[self._airfoils[index].name]+tip_weight*airfoil_outlines[self._airfoils[index+1].name]
                break
            index += 1

        # Add twist, dihedral, and chord
        twist = self.get_twist(span)
        dihedral = self.get_dihedral(span)
        chord = self.get_chord(span)

        # Transform to body-fixed coordinates
        if self._side == "left":
            q = euler_to_quaternion(np.array([dihedral, twist, 0.0]))
        else:
            q = euler_to_quaternion(np.array([-dihedral, twist, 0.0]))

        untransformed_coords = chord*np.array([-points[:,0].flatten()+0.25, np.zeros(N), -points[:,1]]).T
        coords = self._get_quarter_chord_loc(span)[np.newaxis]+quaternion_inverse_transform(q, untransformed_coords)

        # Seal trailing edge
        te = (coords[0]+coords[-1])*0.5
        coords[0] = te
        coords[-1] = te

        return coords


    def create_freecad_stp(self, section_res=200):
        """Creates a FreeCAD part representing a loft of the wing segment.

        Parameters
        ----------
        section_res : int
            Number of outline points to use for the sections. Defaults to 200.

        Returns
        -------
        wing : Part
            A FreeCAD part representing the wing segment.
        """

        # Import necessary modules
        import FreeCAD
        import Part

        # Create sections
        sections = []
        for s_i in self._node_span_locs:
            points = []

            # Get outline points
            outline = self._get_airfoil_outline_coords_at_span(s_i, section_res)

            # Check for wing going to a point
            if np.all(np.all(outline == outline[0,:])):
                #tip = FreeCAD.Base.Vector(*outline[0])
                #points.append(tip)
                #continue
                outline = self._get_airfoil_outline_coords_at_span(s_i-0.000001, section_res)

            # Create outline points
            for point in outline:
                points.append(FreeCAD.Base.Vector(*point))

            # Add to section list
            section_spline = Part.BSplineCurve(points)
            sections.append(section_spline.toShape())

        # Loft
        wing_loft = Part.makeLoft(sections, True, False, False).Faces
        wing_shell = Part.Shell(wing_loft)
        wing_solid = Part.Solid(wing_shell)
        return wing_solid