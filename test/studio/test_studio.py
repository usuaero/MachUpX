"""End-to-end tests for Wingxtra Studio: sizing, analysis, export and the HTTP API."""

import io
import math
import zipfile

import pytest
from fastapi.testclient import TestClient

from wingxtra_studio import Requirements, aero, airfoils, analysis, export, geometry, sizing
from wingxtra_studio.api import app

CONFIGS = ["conventional", "flying_wing", "vtol_quadplane"]


@pytest.fixture(scope="module", params=CONFIGS)
def sized(request):
    design = sizing.size(Requirements(config=request.param))
    return design, analysis.analyze(design)


def test_thin_airfoil_values_match_published_naca_2412():
    s = airfoils.section_properties("2412")
    assert math.degrees(s["aL0"]) == pytest.approx(-2.08, abs=0.05)
    assert s["CmL0"] == pytest.approx(-0.053, abs=0.002)
    assert airfoils.section_properties("0012")["aL0"] == pytest.approx(0.0, abs=1e-9)


def test_airfoil_outline_is_closed_and_has_right_thickness():
    pts = airfoils.outline("0012", 60)
    assert tuple(pts[0]) == tuple(pts[-1]) == (1.0, 0.0)
    assert pts[:, 1].max() - pts[:, 1].min() == pytest.approx(0.12, abs=0.002)


@pytest.mark.parametrize("bad", ["12", "abcd", "2012", "0002"])
def test_invalid_airfoils_are_rejected(bad):
    with pytest.raises(ValueError):
        airfoils.parse(bad)


def test_sizing_meets_requirements_as_predicted_by_the_solver(sized):
    design, a = sized
    req = design.requirements
    assert a["performance"]["stall_speed_ms"] == pytest.approx(req.stall_speed_ms, rel=0.02)
    assert a["performance"]["cruise"]["endurance_min"] == pytest.approx(req.endurance_min, rel=0.03)
    assert a["trim"]["static_margin"] == pytest.approx(req.static_margin, abs=0.02)
    assert a["trim"]["CL"] == pytest.approx(a["trim"]["CL_required"], rel=1e-3)
    assert a["summary"]["fail"] == 0


def test_masses_sum_to_takeoff_mass_and_balance_on_cg(sized):
    design, _ = sized
    total = sum(i.mass_kg for i in design.masses)
    assert total == pytest.approx(design.mass_kg, rel=1e-4)
    assert sum(i.mass_kg * i.x for i in design.masses) / total == pytest.approx(design.cg_x, abs=1e-6)


def test_modes_are_physical(sized):
    _, a = sized
    modes = a["dynamics"]["modes"]
    assert {"short_period", "phugoid", "dutch_roll", "roll", "spiral"} <= set(modes)
    assert modes["short_period"]["stable"] and modes["dutch_roll"]["stable"] and modes["roll"]["stable"]
    assert modes["phugoid"]["period_s"] > modes["short_period"].get("period_s", 0)


def test_moving_cg_aft_reduces_static_margin():
    design = sizing.size(Requirements())
    base = aero.trim_and_derivatives(design, 1.0)["static_margin"]
    design.cg_x -= 0.05 * design.wing.mac
    assert aero.trim_and_derivatives(design, 1.0)["static_margin"] == pytest.approx(base - 0.05, abs=0.004)


def test_more_payload_needs_a_bigger_aircraft():
    light = sizing.size(Requirements(payload_kg=0.5), passes=1)
    heavy = sizing.size(Requirements(payload_kg=2.0), passes=1)
    assert heavy.mass_kg > light.mass_kg and heavy.wing.area > light.wing.area


def test_impossible_requirements_explain_themselves():
    with pytest.raises(sizing.SizingError, match="do not close"):
        sizing.size(Requirements(endurance_min=1400))
    with pytest.raises(sizing.SizingError, match="1.2 times"):
        sizing.size(Requirements(cruise_speed_ms=11, stall_speed_ms=10.5))


def test_step_export_contains_valid_named_solids(sized):
    design, _ = sized
    data = export.step(design)
    text = data.decode("latin-1")
    assert text.startswith("ISO-10303-21") and "PRODUCT('Wing right'" in text

    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    import tempfile, os

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "a.step")
        with open(path, "wb") as fh:
            fh.write(data)
        reader = STEPControl_Reader()
        reader.ReadFile(path)
        reader.TransferRoots()
        explorer, count = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID), 0
        while explorer.More():
            assert BRepCheck_Analyzer(explorer.Current()).IsValid()
            count += 1
            explorer.Next()
    assert count == len(geometry.lofts(design))


def test_stl_and_machupx_exports(sized):
    design, _ = sized
    data = export.stl(design)
    triangles = int.from_bytes(data[80:84], "little")
    assert triangles > 100 and len(data) == 84 + 50 * triangles
    names = zipfile.ZipFile(io.BytesIO(export.machup_zip(design))).namelist()
    assert names == ["airplane.json", "input.json"]


def test_api_round_trip():
    client = TestClient(app)
    meta = client.get("/api/meta").json()
    assert len(meta["presets"]) >= 3

    res = client.post("/api/size", json=meta["presets"][0]["requirements"])
    assert res.status_code == 200
    body = res.json()
    assert set(body) == {"design", "analysis", "mesh"}

    design = body["design"]
    design["wing"]["dihedral_deg"] = 8.0
    again = client.post("/api/analyze", json=design).json()
    assert again["analysis"]["trim"]["stability"]["Cl,b"] < body["analysis"]["trim"]["stability"]["Cl,b"]

    assert client.post("/api/export/step", json=design).content.startswith(b"ISO-10303-21")
    assert client.post("/api/export/nope", json=design).status_code == 404
    assert client.post("/api/size", json={"endurance_min": 1400}).status_code == 422
    assert client.get("/").status_code == 200


# ---------------------------------------------------------------- spreadsheet mode and V-tails

from wingxtra_studio import sheet  # noqa: E402

# Calculated cells of the Wingxtra design sheet for its default inputs.
SHEET_REFERENCE = {
    "total_mass_kg": 15.98571, "battery_kg": 5.28571, "mass_fraction_pct": 43.69973, "wing_area": 0.70298,
    "semispan": 1.49984, "root_chord": 0.36054, "tip_chord": 0.10816, "average_chord": 0.23435,
    "min_drag_airspeed": 16.94858, "min_power_airspeed": 12.87814, "stall_speed": 18.02991, "stall_margin": 4.97009,
    "CL": 0.68825, "CD": 0.06086, "LD": 11.30908, "flight_time_min": 125.62124, "range_km": 173.35731,
    "propulsive_power_w": 398.53209, "cruise_current_a": 8.97595, "h_area": 0.07551, "h_semispan": 0.33654,
    "h_root_chord": 0.11218, "v_area": 0.04793, "v_root_chord": 0.12746, "v_tip_chord": 0.11217,
    "vtail_required_area": 0.12343, "vtail_dihedral_deg": 38.54389, "vtail_span_prediction": 0.93442,
    "vtail_chord_prediction": 0.15574,
}


@pytest.mark.parametrize("key", sorted(SHEET_REFERENCE))
def test_sheet_formulas_reproduce_the_spreadsheet(key):
    assert sheet.estimates(sheet.SheetInputs())[key] == pytest.approx(SHEET_REFERENCE[key], rel=1e-4)


@pytest.mark.parametrize("tail", ["conventional", "v_tail"])
def test_sheet_design_matches_sheet_geometry_and_flies(tail):
    inputs = sheet.SheetInputs(tail=tail)
    design = sheet.build(inputs)
    assert design.mass_kg == pytest.approx(15.98571, rel=1e-4)
    assert design.wing.area == pytest.approx(0.70298, rel=1e-4)
    assert design.wing.span == pytest.approx(2 * 1.49984, rel=1e-4)
    assert sum(m.mass_kg for m in design.masses) == pytest.approx(design.mass_kg, rel=1e-6)

    a = analysis.analyze(design)
    assert a["summary"]["fail"] == 0
    assert a["trim"]["static_margin"] == pytest.approx(inputs.static_margin, abs=0.02)
    # Solver and sheet should broadly agree; a large gap would mean one of them is wrong.
    for row in sheet.comparison(inputs, a):
        assert abs(row["difference_pct"]) < 20, row

    if tail == "v_tail":
        assert design.vtail is None and design.htail.dihedral_deg == pytest.approx(38.54389, rel=1e-4)
        control = a["trim"]["control"]
        assert abs(control["Cn,drudder"]) > 0.02 and abs(control["Cm,delevator"]) > 0.5
        assert a["trim"]["stability"]["Cn,b"] > 0.04


def test_v_tail_in_requirements_mode():
    design = sizing.size(Requirements(tail="v_tail"), passes=1)
    assert design.tail_type == "v_tail" and design.vtail is None
    assert analysis.analyze(design)["summary"]["fail"] == 0
    assert "PRODUCT('V-tail right'" in export.step(design).decode("latin-1")


def test_sheet_api_and_reanalysis_keep_the_comparison():
    client = TestClient(app)
    defaults = client.get("/api/meta").json()["sheet_defaults"]
    body = client.post("/api/sheet", json=defaults).json()
    assert body["design"]["source"] == "spreadsheet"
    assert body["sheet"]["estimates"]["total_mass_kg"] == pytest.approx(15.98571, rel=1e-4)

    design = body["design"]
    design["wing"]["washout_deg"] = -4.0
    again = client.post("/api/analyze", json=design).json()
    assert "comparison" in again["sheet"]

    too_slow = dict(defaults, cruise_airspeed_ms=15)
    assert client.post("/api/sheet", json=too_slow).status_code == 422
