from .helpers import _check_filepath,_vectorized_convert_units,_import_value

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
            self._splines = {}
            self._initialize_params()
            self._initialize_getters()
            self._initialize_airfoils(airfoil_dict)

    
    def _initialize_params(self):

        # Set global params
        self.is_main = self._input_dict.get("is_main")
        self.b = _import_value("span", self._input_dict, self._unit_sys, -1)
        self._N = self._input_dict.get("grid", 40)
        self._use_clustering = self._input_dict.get("use_clustering", True)

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
        if self._use_clustering: # Cosine clustering
            #Nodes
            node_theta_space = np.linspace(0.0, np.pi, self._N+1)
            self._node_span_locs = (1-np.cos(node_theta_space))/2

            # Control points
            cp_theta_space = np.linspace(np.pi/self._N, np.pi, self._N)-np.pi/(2*self._N)
            self._cp_span_locs = (1-np.cos(cp_theta_space))/2

        else: # Linear spacing
            self._node_span_locs = np.linspace(0.0, 1.0, self._N+1)
            self._cp_span_locs = np.linspace(1/(2*self._N), 1.0-1/(2*self._N), self._N)

        # In order to follow the airfoil sign convention (i.e. positive vorticity creates positive lift) 
        # node and control point locations must always proceed from left to right.
        if self._side == "left":
            self._node_span_locs = self._node_span_locs[::-1]
            self._cp_span_locs = self._cp_span_locs[::-1]


    def _initialize_getters(self):
        # Sets getters for functions which are a function of span
        twist_data = _import_value("twist", self._input_dict, self._unit_sys, 0)
        self.get_twist = self._build_getter_linear_f_of_span(twist_data, "twist", angular_data=True)

        dihedral_data = _import_value("dihedral", self._input_dict, self._unit_sys, 0)
        self.get_dihedral = self._build_getter_linear_f_of_span(dihedral_data, "dihedral", angular_data=True)

        sweep_data = _import_value("sweep", self._input_dict, self._unit_sys, 0)
        self.get_sweep = self._build_getter_linear_f_of_span(sweep_data, "sweep", angular_data=True)

        chord_data = _import_value("chord", self._input_dict, self._unit_sys, 1.0)
        self.get_chord = self._build_getter_linear_f_of_span(chord_data, "chord")

        ac_offset_data = _import_value("ac_offset", self._input_dict, self._unit_sys, 0)
        self._get_ac_offset = self._build_getter_linear_f_of_span(ac_offset_data, "ac_offset")

        ## Setup quarter-chord position getters

        #num_samples = 10
        #x_samples = np.zeros(num_samples)
        #y_samples = np.zeros(num_samples)
        #z_samples = np.zeros(num_samples)
        #span_locs = np.linspace(0.0, 1.0, num_samples)

        #for i, span in enumerate(span_locs):
        #    x_samples[i] = integ.quad(lambda s : -np.tan(np.radians(self.get_sweep(s))), 0, span)[0]*self.b
        #    if self._side == "left":
        #        y_samples[i] = integ.quad(lambda s : -np.cos(np.radians(self.get_dihedral(s))), 0, span)[0]*self.b
        #    else:
        #        y_samples[i] = integ.quad(lambda s : np.cos(np.radians(self.get_dihedral(s))), 0, span)[0]*self.b
        #    z_samples[i] = integ.quad(lambda s : -np.sin(np.radians(self.get_dihedral(s))), 0, span)[0]*self.b

        #self._get_qc_dx_loc = interp.interp1d(span_locs, x_samples, kind="cubic")
        #self._get_qc_dy_loc = interp.interp1d(span_locs, y_samples, kind="cubic")
        #self._get_qc_dz_loc = interp.interp1d(span_locs, z_samples, kind="cubic")


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

    def _initialize_airfoils(self, airfoil_dict):
        # Picks out the airfoils used in this wing segment and stores them. Also 
        # initializes airfoil coefficient getters

        # Get which airfoils are specified for this segment
        default_airfoil = list(airfoil_dict.keys())[0]
        airfoil = _import_value("airfoil", self._input_dict, self._unit_sys, default_airfoil)

        self._airfoils = []
        self._airfoil_spans = []
        self._num_airfoils = 0

        # Setup data table
        if isinstance(airfoil, str): # Constant airfoil

            if not airfoil in list(airfoil_dict.keys()):
                raise IOError("'{0}' must be specified in 'airfoils'.".format(airfoil))

            self._airfoils.append(airfoil_dict[airfoil])
            self._airfoils.append(airfoil_dict[airfoil])
            self._airfoil_spans.append(0.0)
            self._airfoil_spans.append(1.0)
            self._num_airfoils = 2


        elif isinstance(airfoil, np.ndarray): # Distribution of airfoils
            self._airfoil_data = np.empty((airfoil.shape[0], airfoil.shape[1]+1), dtype=None)

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

        self._CLa = []
        self._aL0 = []
        for airfoil in self._airfoils:
            self._CLa.append(airfoil.get_CLa())
            self._aL0.append(airfoil.get_aL0())

        self._CLa = np.asarray(self._CLa)
        self._aL0 = np.asarray(self._aL0)


    def get_CLa(self, span):
        """Returns the lift slope as a function of span. Used for the linear 
        solution to NLL.

        Parameters
        ----------
        span : float
            Span location(s)

        Returns
        -------
        float
            Lift slope
        """
        return np.interp(span, self._airfoil_spans, self._CLa)


    def get_aL0(self, span):
        """Returns the zero-lift angle of attack as a function of span. Used for the linear 
        solution to NLL.

        Parameters
        ----------
        span : float
            Span location(s)

        Returns
        -------
        float
            Zero lift angle of attack
        """
        return np.interp(span, self._airfoil_spans, self._aL0)

    
    def get_cp_CLa(self):
        """Returns the lift slope at each control point.

        Returns
        -------
        ndarray
            Array of lift slopes at each control point.
        """
        return self.get_CLa(self._cp_span_locs)

    
    def get_cp_aL0(self):
        """Returns the zero-lift angle of attack at each control point.

        Returns
        -------
        ndarray
            Array of zero-lift angles of attack at each control point.
        """
        return self.get_aL0(self._cp_span_locs)


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

        # This can only be called by the origin segment
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
        # Returns a reference to the specified wing segment. ONLY FOR TESTING!
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

        Parameters
        ----------

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

        Parameters
        ----------

        Returns
        -------
        ndarray
            Location of the tip quarter-chord.
        """
        if self.ID == 0:
            return self._origin
        else:
            return self.get_quarter_chord_loc(1.0)


    def get_quarter_chord_loc(self, span):
        """Returns the location of the quarter-chord at the given span fraction.

        Parameters
        ----------
        span : float
            Span location as a fraction of the total span starting at the root.

        Returns
        -------
        ndarray
            Location of the quarter-chord.
        """
        if isinstance(span, float):
            converted = True
            span_array = np.asarray(span)[np.newaxis]
        else:
            converted = False
            span_array = np.asarray(span)

        ds = np.zeros((span_array.shape[0],3))
        for i, span in enumerate(span_array):
            ds[i,0] = integ.quad(lambda s : -np.tan(self.get_sweep(s)), 0, span)[0]*self.b
            if self._side == "left":
                ds[i,1] = integ.quad(lambda s : -np.cos(self.get_dihedral(s)), 0, span)[0]*self.b
            else:
                ds[i,1] = integ.quad(lambda s : np.cos(self.get_dihedral(s)), 0, span)[0]*self.b
            ds[i,2] = integ.quad(lambda s : -np.sin(self.get_dihedral(s)), 0, span)[0]*self.b

        #ds[0] = self._get_qc_dx_loc(span_array)
        #ds[1] = self._get_qc_dy_loc(span_array)
        #ds[2] = self._get_qc_dz_loc(span_array)

        qc_loc = self.get_root_loc()+ds
        if converted:
            qc_loc = qc_loc.flatten()

        return qc_loc


    def get_section_ac_loc(self, span):
        """Returns the location of the section aerodynamic center at the given span fraction.

        Parameters
        ----------
        span : float
            Span location as a fraction of the total span starting at the root.

        Returns
        -------
        ndarray
            Location of the section aerodynamic center.
        """
        loc = self.get_quarter_chord_loc(span)
        loc += self._get_ac_offset(span)[:,np.newaxis]*self._get_axial_vec(span)
        return loc


    def _get_axial_vec(self, span):
        # Returns the axial vector at the given span locations
        if isinstance(span, float):
            span_array = np.asarray(span)[np.newaxis]
        else:
            span_array = np.asarray(span)

        twist = self.get_twist(span_array)
        
        C_twist = np.cos(twist)
        S_twist = np.sin(twist)

        return np.asarray([-C_twist, np.zeros(span_array.shape[0]), S_twist]).T


    def get_CL(self, span, inputs):
        """Returns the coefficient of lift at the given span location as a function of *args.

        Parameters
        ----------
        span : float or ndarray
            Span location as a fraction of the total span, starting at the root.

        inputs : ndarray
            Airfoil parameters. The first is always angle of attack in radians. If a 2D 
            array, specifies the airfoil parameters at each given span location.

        Returns
        -------
        float or ndarray
            Coefficient of lift
        """

        if isinstance(span, float):
            span_array = np.asarray(span)[np.newaxis]
            input_array = inputs[np.newaxis]
        else:
            if span.shape[0] != inputs.shape[0]:
                raise ValueError("""span and inputs must have the same first dimension. Could 
                                    not match dimensions {0} and {1}.""".format(span.shape, inputs.shape))
            span_array = np.copy(span)
            input_array = np.copy(inputs)

        print(span_array.shape)
        CLs = np.zeros((span_array.shape[0],self._num_airfoils))
        for i in range(self._num_airfoils):
            CLs[:,i] = self._airfoils[i].get_CL(input_array)

        return np.interp(span_array[np.newaxis], self._airfoil_spans[np.newaxis], CLs)


    def get_CD(self, span, *args):
        """Returns the coefficient of drag at the given span location as a function of *args.

        Parameters
        ----------
        span : float
            Span location as a fraction of the total span, starting at the root.

        *args : floats
            Airfoil parameters. The first is always angle of attack in radians.

        Returns
        -------
        float
            Coefficient of drag
        """
        if len(self._airfoils) == 1:
            CD = self._airfoils[0].get_CD(*args)
        
        else:
            for i in range(len(self._airfoils)):
                if span >= self._airfoil_spans[i] and span <= self._airfoil_spans[i+1]:
                    break

            s0 = self._airfoil_spans[i]
            CD0 = self._airfoils[i].get_CD(*args)
            s1 = self._airfoil_spans[i+1]
            CD1 = self._airfoils[i+1].get_CD(*args)

            CD = CD0 + span*(CD1-CD0)/(s1-s0)

        return CD


    def get_Cm(self, span, *args):
        """Returns the moment coefficient at the given span location as a function of *args.

        Parameters
        ----------
        span : float
            Span location as a fraction of the total span, starting at the root.

        *args : floats
            Airfoil parameters. The first is always angle of attack in radians.

        Returns
        -------
        float
            Moment coefficient
        """
        if len(self._airfoils) == 1:
            Cm = self._airfoils[0].get_Cm(*args)
        
        else:
            for i in range(len(self._airfoils)):
                if span >= self._airfoil_spans[i] and span <= self._airfoil_spans[i+1]:
                    break

            s0 = self._airfoil_spans[i]
            Cm0 = self._airfoils[i].get_Cm(*args)
            s1 = self._airfoil_spans[i+1]
            Cm1 = self._airfoils[i+1].get_Cm(*args)

            Cm = Cm0 + span*(Cm1-Cm0)/(s1-s0)

        return Cm


    def get_node_locs(self):
        """Returns the location of all horseshoe vortex node pairs on the segment.

        Returns
        -------
        ndarray
            Array of horseshoe vortex node pairs. First index is the node and 
            second index is the position component.
        """
        return self.get_section_ac_loc(self._node_span_locs)


    def get_cp_locs(self):
        """Returns the location of all control points on the segment.

        Returns
        -------
        ndarray
            Array of control points placed on the quarter-chord.
        """
        return self.get_section_ac_loc(self._cp_span_locs)


    def get_cp_chord_lengths(self):
        """Returns the local chord length at each control point on the segment.

        Returns
        -------
        ndarray
            Array of chord lengths corresponding to each control point.
        """
        return self.get_chord(self._cp_span_locs)


    def get_cp_normal_vecs(self):
        """Returns the local normal vector at each control point on the segment.

        Returns
        ----------
        ndarray
            Array of normal vectors. First index is the control point index and second 
            index is the vector component.
        """
        twist = self.get_twist(self._cp_span_locs)
        dihedral = self.get_dihedral(self._cp_span_locs)
        
        C_twist = np.cos(twist)
        S_twist = np.sin(twist)
        C_dihedral = np.cos(dihedral)
        S_dihedral = np.sin(dihedral)

        if self._side == "left":
            normal_vecs = np.asarray([-S_twist*C_dihedral, S_dihedral, -C_twist*C_dihedral]).T
        else:
            normal_vecs = np.asarray([-S_twist*C_dihedral, -S_dihedral, -C_twist*C_dihedral]).T


        return normal_vecs


    def get_cp_axial_vecs(self):
        """Returns the local axial vector at each control point on the segment.

        Returns
        ----------
        ndarray
            Array of axial vectors. First index is the control point index and second 
            index is the vector component.
        """
        return self._get_axial_vec(self._cp_span_locs)

    
    def get_cp_span_vecs(self):
        """Returns the local spanwise vector at each control point on the segment.

        Returns
        -------
        ndarray
            Array of span vectors. First index is the control point index and second 
            index is the vector component.
        """
        dihedral = self.get_dihedral(self._cp_span_locs)

        C_dihedral = np.cos(dihedral)
        S_dihedral = np.sin(dihedral)

        if self._side == "left":
            return np.asarray([np.zeros(self._N), -C_dihedral, -S_dihedral]).T
        else:
            return np.asarray([np.zeros(self._N), C_dihedral, -S_dihedral]).T


    def get_cp_avg_chord_lengths(self):
        """Returns the average local chord length at each control point on the segment.

        Returns
        -------
        ndarray
            Array of chord lengths corresponding to each control point.
        """
        node_chords = self.get_chord(self._node_span_locs)
        return (node_chords[1:]+node_chords[:-1])/2


    def get_array_of_dS(self):
        """Returns the differential elements of area.

        Returns
        -------
        ndarray
            Array of area differential elements.
        """
        ds = abs(self._node_span_locs[1:]-self._node_span_locs[:-1])*self.b
        return self.get_cp_avg_chord_lengths()*ds


    def get_outline_points(self):
        """Returns a set of points that represents the outline of the wing segment.
        
        Returns
        -------
        ndarray
            Array of outline points.
        """
        num_span_locs = 10
        spans = np.linspace(0, 1, num_span_locs)
        qc_points = self.get_quarter_chord_loc(spans)
        chords = self.get_chord(spans)
        axial_vecs = self._get_axial_vec(spans)

        points = np.zeros((num_span_locs*2+1,3))

        # Leading edge
        points[:num_span_locs,:] = qc_points - 0.25*(axial_vecs*chords[:,np.newaxis])

        # Trailing edge
        points[-2:num_span_locs-1:-1,:] = qc_points - 0.75*(axial_vecs*chords[:,np.newaxis])

        # Complete the circle
        points[-1,:] = points[0,:]

        return points
