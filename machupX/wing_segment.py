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

    def __init__(self, name, input_dict, side, unit_sys, airfoil_dict, origin=[0,0,0]):

        self.name = name
        self._input_dict = input_dict
        self._unit_sys = unit_sys
        self._side = side
        self._origin = np.asarray(origin).reshape((3,1))

        self._attached_segments = {}
        
        self.ID = self._input_dict.get("ID")
        if self.ID == 0 and name != "origin":
            raise IOError("Wing segment ID may not be 0.")

        if self.ID != 0: # These do not need to be run for the origin segment
            self._splines = {}
            self._initialize_params()
            self._initialize_getters()
            self._initialize_airfoils(airfoil_dict)

    
    def _initialize_params(self):

        # Set global params
        self.is_main = self._input_dict.get("is_main")
        self.b = _import_value("span", self._input_dict, self._unit_sys, -1)
        self.N = self._input_dict.get("grid", 40)
        self._use_clustering = self._input_dict.get("use_clustering", True)

        self._delta_origin = np.zeros((3,1))
        connect_dict = self._input_dict.get("connect_to", {})
        self._delta_origin[0] = connect_dict.get("dx", 0.0)
        self._delta_origin[1] = connect_dict.get("dy", 0.0)
        self._delta_origin[2] = connect_dict.get("dz", 0.0)

        if self._side == "left":
            self._delta_origin[1] -= connect_dict.get("y_offset", 0.0)
        else:
            self._delta_origin[1] += connect_dict.get("y_offset", 0.0)

        if self._use_clustering:
            self._node_span_locs = (1-np.cos(np.linspace(0.0, np.pi, self.N+1)))/2
            self._cp_span_locs = (1-np.cos(np.linspace(0.0, np.pi, self.N)+np.pi/(2*self.N)))/2
        else:
            self._node_span_locs = np.linspace(0.0, 1.0, self.N+1)
            self._cp_span_locs = np.linspace(1/(2*self.N), 1.0-1/(2*self.N), self.N)


    def _initialize_getters(self):
        # Sets getters for functions which are a function of span
        twist_data = _import_value("twist", self._input_dict, self._unit_sys, 0)
        self.get_twist = self._build_getter_linear_f_of_span(twist_data, "twist")

        dihedral_data = _import_value("dihedral", self._input_dict, self._unit_sys, 0)
        self.get_dihedral = self._build_getter_linear_f_of_span(dihedral_data, "dihedral")

        sweep_data = _import_value("sweep", self._input_dict, self._unit_sys, 0)
        self.get_sweep = self._build_getter_linear_f_of_span(sweep_data, "sweep")

        chord_data = _import_value("chord", self._input_dict, self._unit_sys, 1.0)
        self.get_chord = self._build_getter_linear_f_of_span(chord_data, "chord")

        ac_offset_data = _import_value("ac_offset", self._input_dict, self._unit_sys, 0)
        self._get_ac_offset = self._build_getter_linear_f_of_span(ac_offset_data, "ac_offset")

        # Setup quarter-chord position getters
        num_samples = 100
        x_samples = np.zeros(100)
        y_samples = np.zeros(100)
        z_samples = np.zeros(100)
        span_locs = np.linspace(0.0, 1.0, num_samples)

        for i, span in enumerate(span_locs):
            x_samples[i] = integ.quad(lambda s : -np.tan(np.radians(self.get_sweep(s))), 0, span)[0]*self.b
            if self._side == "left":
                y_samples[i] = integ.quad(lambda s : -np.cos(np.radians(self.get_dihedral(s))), 0, span)[0]*self.b
            else:
                y_samples[i] = integ.quad(lambda s : np.cos(np.radians(self.get_dihedral(s))), 0, span)[0]*self.b
            z_samples[i] = integ.quad(lambda s : -np.sin(np.radians(self.get_dihedral(s))), 0, span)[0]*self.b

        self._get_qc_dx_loc = interp.interp1d(span_locs, x_samples, kind="cubic")
        self._get_qc_dy_loc = interp.interp1d(span_locs, y_samples, kind="cubic")
        self._get_qc_dz_loc = interp.interp1d(span_locs, z_samples, kind="cubic")


    def _build_getter_linear_f_of_span(self, data, name):
        # Defines a getter function for data which is a function of span

        if isinstance(data, float): # Constant
            return interp.interp1d(np.asarray([0.0, 1.0]), np.asarray([data, data]), kind="linear")
        
        else: # Array
            return interp.interp1d(data[:,0], data[:,1], kind="linear")


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
            span_array = np.asarray(span)[np.newaxis]
        else:
            span_array = np.asarray(span)

        ds = np.zeros((3,span_array.shape[0]))
        ds[0] = self._get_qc_dx_loc(span_array)
        ds[1] = self._get_qc_dy_loc(span_array)
        ds[2] = self._get_qc_dz_loc(span_array)

        return self.get_root_loc()+ds


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
        loc[0] += self._get_ac_offset(span)
        return loc


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


    def get_CL(self, span, *args):
        """Returns the coefficient of lift at the given span location as a function of *args.

        Parameters
        ----------
        span : float
            Span location as a fraction of the total span, starting at the root.

        *args : floats
            Airfoil parameters. The first is always angle of attack in radians.

        Returns
        -------
        float
            Coefficient of lift
        """

        CLs_for_spline = np.zeros(self._num_airfoils)
        for i in range(self._num_airfoils):
            CLs_for_spline[i] = self._airfoils[i].get_CL(*args)

        CL_spline = interp.interp1d(self._airfoil_spans, CLs_for_spline, kind="linear")

        return CL_spline(span)


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

        Parameters
        ----------

        Returns
        -------
        ndarray
            Array of horseshoe vortex node pairs. First index is the vortex, second
            index is which node, and third index is the position components.
        """
        node_spans = np.asarray([self._node_span_locs[:-1],self._node_span_locs[1:]])

        return self.get_section_ac_loc(node_spans)
