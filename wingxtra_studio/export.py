"""Hand-off to other tools: STEP solids for CAD, STL for printing and CFD, MachUpX input."""

import contextlib
import io
import json
import os
import struct
import sys
import tempfile
import zipfile
from typing import Dict, List

import numpy as np

from . import aero, geometry
from .schema import Design

MM = 1000.0


def stl(design: Design) -> bytes:
    """Binary STL in millimetres."""
    tris = []
    for loft in geometry.lofts(design):
        verts, idx = geometry._triangulate(loft["rings"])
        tris.append(verts[np.array(idx).reshape(-1, 3)] * MM)
    t = np.vstack(tris)
    normals = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)
    buf = io.BytesIO()
    buf.write(b"Wingxtra Studio".ljust(80, b" "))
    buf.write(struct.pack("<I", len(t)))
    for n, tri in zip(normals, t):
        buf.write(struct.pack("<12fH", *n, *tri.ravel(), 0))
    return buf.getvalue()


# ---------------------------------------------------------------- STEP

@contextlib.contextmanager
def _quiet():
    """OpenCASCADE prints transfer statistics straight to the process's stdout."""
    sys.stdout.flush()
    saved = os.dup(1)
    with open(os.devnull, "w") as devnull:
        os.dup2(devnull.fileno(), 1)
        try:
            yield
        finally:
            os.dup2(saved, 1)
            os.close(saved)


def _spline(points: np.ndarray, periodic: bool):
    """An interpolating B-spline edge through the points (metres in, millimetres out)."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
    from OCP.GeomAPI import GeomAPI_Interpolate
    from OCP.gp import gp_Pnt

    try:  # OpenCASCADE 8 renamed its collection classes
        from OCP.collections import HArray1_gp_Pnt as PointArray
    except ImportError:
        from OCP.TColgp import TColgp_HArray1OfPnt as PointArray

    arr = PointArray(1, len(points))
    for i, p in enumerate(points):
        arr.SetValue(i + 1, gp_Pnt(*(float(v) * MM for v in p)))
    interp = GeomAPI_Interpolate(arr, periodic, 1e-6)
    interp.Perform()
    return BRepBuilderAPI_MakeEdge(interp.Curve()).Edge()


def _spline_edge(points: np.ndarray):
    return _spline(points, False)


def _closed_spline_wire(ring: np.ndarray):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeWire

    return BRepBuilderAPI_MakeWire(_spline(ring, True)).Wire()


def _airfoil_wire(ring: np.ndarray):
    """Upper and lower splines meeting at a sharp trailing edge and at the leading edge."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeWire

    le = len(ring) // 2
    wire = BRepBuilderAPI_MakeWire()
    wire.Add(_spline_edge(ring[: le + 1]))
    wire.Add(_spline_edge(ring[le:]))
    return wire.Wire()


def _solid(loft: Dict):
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections

    builder = BRepOffsetAPI_ThruSections(True, loft["ruled"], 1e-6)
    airfoil = np.allclose(loft["rings"][0][0], loft["rings"][0][-1])
    for ring in loft["rings"]:
        builder.AddWire(_airfoil_wire(ring) if airfoil else _closed_spline_wire(ring))
    builder.Build()
    return builder.Shape()


def step(design: Design) -> bytes:
    """A STEP AP214 assembly, one named solid per component, in millimetres.

    Axes follow the aircraft body frame: x forward, y to the right wing, z down.
    """
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    for loft in geometry.lofts(design):
        label = shapes.AddShape(_solid(loft), False)
        TDataStd_Name.Set_s(label, TCollection_ExtendedString(loft["name"]))

    Interface_Static.SetCVal_s("write.step.schema", "AP214")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    with tempfile.TemporaryDirectory() as tmp, _quiet():
        writer = STEPCAFControl_Writer()
        writer.SetNameMode(True)
        writer.Transfer(doc, STEPControl_AsIs)
        path = os.path.join(tmp, "aircraft.step")
        if writer.Write(path) != IFSelect_RetDone:
            raise RuntimeError("STEP export failed.")
        with open(path, "rb") as fh:
            return fh.read()


# ---------------------------------------------------------------- MachUpX

def machup_zip(design: Design) -> bytes:
    """airplane.json and input.json, runnable with `python -m machupX input.json`."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("airplane.json", json.dumps(aero.machup_airplane(design), indent=4))
        z.writestr("input.json", json.dumps(aero.machup_scene_input(design), indent=4))
    return buf.getvalue()


FORMATS: Dict[str, Dict] = {
    "step": {"fn": step, "ext": "step", "mime": "model/step", "label": "STEP solid model"},
    "stl": {"fn": stl, "ext": "stl", "mime": "model/stl", "label": "STL mesh"},
    "machupx": {"fn": machup_zip, "ext": "zip", "mime": "application/zip", "label": "MachUpX input files"},
    "json": {"fn": lambda d: d.model_dump_json(indent=2).encode(), "ext": "json", "mime": "application/json",
             "label": "Design file"},
}
