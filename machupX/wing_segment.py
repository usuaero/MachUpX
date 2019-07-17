from .helpers import _check_filepath,_vectorized_convert_units,_import_value

import json
import numpy as np

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

    def __init__(self, name, input_dict, side, unit_sys, origin=[0,0,0]):

        self.name = name
        self._input_dict = input_dict
        self._unit_sys = unit_sys
        self._side = side
        self._origin = np.asarray(origin)

        self._attached_segments = {}
        
        self.ID = self._input_dict.get("ID")
        if self.ID == 0 and name != "origin":
            raise IOError("Wing segment ID may not be 0.")

        if self.ID != 0: # These do not need to be run for the origin segment
            self._getter_data = {}
            self._initialize_params()
            self._initialize_getters()

    
    def _initialize_params(self):

        # Set global params
        self.is_main = self._input_dict.get("is_main")
        self.b = _import_value("span", self._input_dict, self._unit_sys, -1)
        self.N = self._input_dict.get("grid", 40)
        self._use_clustering = self._input_dict.get("use_clustering", True)

        self._delta_origin = np.zeros(3)
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
        self.get_twist = self._build_getter_f_of_span(twist_data, "twist")

        dihedral_data = _import_value("dihedral", self._input_dict, self._unit_sys, 0)
        self.get_dihedral = self._build_getter_f_of_span(dihedral_data, "dihedral")

        sweep_data = _import_value("sweep", self._input_dict, self._unit_sys, 0)
        self.get_sweep = self._build_getter_f_of_span(sweep_data, "sweep")

        chord_data = _import_value("chord", self._input_dict, self._unit_sys, 0)
        self.get_chord = self._build_getter_f_of_span(chord_data, "chord")


    def _build_getter_f_of_span(self, data, name):
        # Defines a getter function for data which is a function of span
        self._getter_data[name] = data

        if isinstance(data, float): # Constant
            def getter(span):
                return self._getter_data[name]
        
        else: # Array
            def getter(span):
                return np.interp(span, self._getter_data[name][0], self._getter_data[name][1])


    def get_root_loc(self):
        # Returns the location of the root quarter-chord
        if self.ID == 0:
            return self._origin
        else:
            return self._origin+self._delta_origin

    def get_tip_loc(self):
        # Returns the location of the tip quarter-chord
        if self.ID == 0:
            return self._origin
        else:
            return self._origin+self._delta_origin

    def attach_wing_segment(self, wing_segment_name, input_dict, side, unit_sys):
        # Attaches a wing segment. Uses tree recursion.

        parent_ID = input_dict.get("connect_to", {}).get("ID", 0)
        if self.ID == parent_ID: # The new segment is supposed to attach to this one

            # Determine the connection point
            if input_dict.get("connect_to", {}).get("location", "tip") == "root":
                attachment_point = self.get_root_loc()
            else:
                attachment_point = self.get_tip_loc()

            self._attached_segments[wing_segment_name] = WingSegment(wing_segment_name, input_dict, side, unit_sys, attachment_point)

            return self._attached_segments[wing_segment_name] # Return reference to newly created wing segment

        else: # We need to recurse deeper
            result = False
            for key in self._attached_segments:
                if side not in key: # Only attach segments of the same side
                    continue
                result = self._attached_segments[key].attach_wing_segment(wing_segment_name, input_dict, side, unit_sys)
                if result is not False:
                    break

            if self.ID == 0 and not result:
                raise RuntimeError("Could not attach wing segment {0}. Check ID of parent is valid.".format(wing_segment_name))

            return result