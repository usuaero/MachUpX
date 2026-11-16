"""HTTP API and static web app.  Run with:  python -m wingxtra_studio"""

import os
import re
import warnings
from typing import Dict

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

from . import __version__, airfoils, analysis, export, geometry, sizing
from .schema import Design, Requirements

warnings.filterwarnings("ignore")  # MachUpX is chatty about benign conditions

WEB = os.path.join(os.path.dirname(__file__), "web")

PRESETS = [
    {"id": "mapper", "label": "Survey fixed-wing", "blurb": "1 kg camera, one hour",
     "requirements": Requirements(name="Survey fixed-wing").model_dump()},
    {"id": "wing", "label": "Flying wing", "blurb": "Compact, hand-launched",
     "requirements": Requirements(name="Flying wing", config="flying_wing", payload_kg=0.5, aspect_ratio=6.5,
                                  taper_ratio=0.5, wing_airfoil="2410", cruise_speed_ms=17, stall_speed_ms=10,
                                  endurance_min=75).model_dump()},
    {"id": "vtol", "label": "VTOL quadplane", "blurb": "2 kg delivery, no runway",
     "requirements": Requirements(name="VTOL quadplane", config="vtol_quadplane", payload_kg=2.0,
                                  cruise_speed_ms=22, stall_speed_ms=14, endurance_min=60, aspect_ratio=9,
                                  hover_time_min=3).model_dump()},
    {"id": "endurance", "label": "Long endurance", "blurb": "Three hours on station",
     "requirements": Requirements(name="Long endurance", payload_kg=0.8, cruise_speed_ms=16, stall_speed_ms=10,
                                  endurance_min=180, aspect_ratio=12, taper_ratio=0.5, wing_airfoil="4412",
                                  battery_wh_per_kg=230, structure_fraction=0.28).model_dump()},
]

app = FastAPI(title="Wingxtra Studio", version=__version__)


def _bundle(design: Design) -> Dict:
    try:
        results = analysis.analyze(design)
    except Exception as exc:  # the solver could not handle this geometry
        raise HTTPException(422, f"The analysis could not be completed for this geometry: {exc}") from exc
    return {"design": design.model_dump(), "analysis": results, "mesh": geometry.mesh(design)}


@app.get("/api/meta")
def meta() -> Dict:
    return {
        "version": __version__,
        "defaults": Requirements().model_dump(),
        "presets": PRESETS,
        "airfoils": airfoils.library(),
        "exports": [{"id": k, "label": v["label"], "ext": v["ext"]} for k, v in export.FORMATS.items()],
    }


@app.post("/api/size")
def size(req: Requirements) -> Dict:
    """Requirements in, a sized and fully analysed aircraft out."""
    try:
        design = sizing.size(req)
    except (sizing.SizingError, ValueError) as exc:
        raise HTTPException(422, str(exc)) from exc
    return _bundle(design)


@app.post("/api/analyze")
def analyze(design: Design) -> Dict:
    """Re-analyse a design after manual edits."""
    try:
        airfoils.parse(design.wing.airfoil)
        for s in (design.htail, design.vtail):
            if s:
                airfoils.parse(s.airfoil)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    sizing.rebalance(design)
    return _bundle(design)


@app.post("/api/export/{fmt}")
def export_design(fmt: str, design: Design) -> Response:
    spec = export.FORMATS.get(fmt)
    if not spec:
        raise HTTPException(404, f"Unknown export format '{fmt}'.")
    try:
        data = spec["fn"](design)
    except Exception as exc:
        raise HTTPException(500, f"Export failed: {exc}") from exc
    name = re.sub(r"[^A-Za-z0-9_-]+", "_", design.requirements.name).strip("_") or "aircraft"
    return Response(data, media_type=spec["mime"],
                    headers={"Content-Disposition": f'attachment; filename="{name}.{spec["ext"]}"'})


@app.get("/")
def index() -> FileResponse:
    return FileResponse(os.path.join(WEB, "index.html"))


app.mount("/", StaticFiles(directory=WEB), name="web")
