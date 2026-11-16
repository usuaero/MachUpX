# Wingxtra Studio

Design a fixed-wing or VTOL UAV from a handful of mission numbers, see how it is predicted
to fly before anything is built, and hand the geometry to CAD.

"Wingxtra Studio" is a working name. The aerodynamic core is
[MachUpX](../README.md) (MIT licence, Utah State University AeroLab), which lives unchanged
in this repository.

## Run it

```bash
pip install -r wingxtra_studio/requirements.txt
```

```bash
python -m wingxtra_studio --port 8000
```

Then open http://127.0.0.1:8000. Tests: `python -m pytest test/studio`.

## What it does today

| Step | What happens |
|---|---|
| Requirements | Payload, cruise and stall speed, endurance, altitude, wing shape, technology assumptions. Three layouts: conventional fixed-wing, flying wing, VTOL quadplane; conventional or V-tail. |
| Spreadsheet mode | The Wingxtra design sheet, row for row: you choose wing loading, battery pack and airframe weight directly. The sheet's formulas are reproduced exactly, the same aircraft is built and run through the solver, and the two sets of numbers are shown side by side. Includes the sheet's V-tail conversion. |
| Sizing | Take-off mass, wing area, tail, battery, cruise motor and lift rotors are solved together. The loop is closed on the solver: the aircraft is resized until the *predicted* stall speed and endurance meet the requirement. |
| Balance | Every mass is placed; the battery position is solved so the centre of gravity gives the requested static margin. Inertia is estimated from the layout. |
| Aerodynamics | MachUpX nonlinear lifting-line: lift curve, drag polar, spanwise loading, critical-section stall, plus a parasite drag build-up for the body, booms and stopped rotors. |
| Stability and control | Cruise trim, neutral point, all stability, damping and control derivatives. |
| Performance | Power curves, endurance, range, climb, glide, maximum speed, hover power and reserve. |
| Flight dynamics | Short period, phugoid, Dutch roll, roll and spiral modes, and time responses to elevator and aileron inputs. |
| Design review | Pass / review / fail verdicts against small-UAV practice and MIL-F-8785C Level 1 boundaries, each with advice. |
| Editing | Any surface dimension can be changed after sizing and is re-analysed immediately. |
| Export | STEP (named lofted solids, mm), STL, MachUpX input files, and a reopenable design file. |

## Layout

```
wingxtra_studio/
  schema.py        Requirements and Design data model
  airfoils.py      NACA 4-digit geometry and section properties
  atmosphere.py    Standard atmosphere, Reynolds number, skin friction
  sizing.py        Requirements -> balanced Design (closed loop on the solver)
  sheet.py         Spreadsheet inputs -> Design, sheet formulas, sheet-vs-solver comparison
  aero.py          Design -> MachUpX model; polar, trim, derivatives, drag build-up
  performance.py   Point performance and hover
  dynamics.py      Linearised equations of motion, modes, responses
  checks.py        Design review rules
  geometry.py      Section lofts and viewer mesh
  export.py        STEP, STL, MachUpX
  analysis.py      Runs everything
  api.py           FastAPI server; also serves web/
  web/             Browser app (no build step; three.js vendored)
```

Conventions follow MachUpX: SI units, x forward, y to the right wing, z down, datum at the
wing-root quarter chord.

## How far to trust the numbers

Lifting-line theory is accurate for attached flow on wings of aspect ratio above about 4,
which covers cruise, trim and stability of most UAVs. Known limits of this version:

- **Airfoil data are estimates.** Zero-lift angle and pitching moment are from thin-airfoil
  theory (good). Maximum lift and profile drag are empirical fits with a Reynolds-number
  correction (indicative, roughly ±10 to 15%). A viscous-solver database replaces them next.
- **Only NACA 4-digit sections** so far; no reflexed or custom airfoils.
- **No propeller slipstream, no fuselage lift or moment**, no ground effect.
- **Stall is the onset of stall**, found where the first wing section reaches its maximum
  lift. Post-stall behaviour is not modelled.
- **The wake is fixed in the body plane** (as AVL does) so results stay smooth; surfaces
  lying exactly in the wing wake are nudged a few millimetres in the solver model only.
- **Lag-of-downwash damping** is taken as one third of pitch damping for tailed aircraft.
- **Structure mass is a fraction of take-off mass** and inertia is estimated; both should be
  replaced by CAD values as the design matures.
- **Hover** uses momentum theory with a figure of merit; transition is not simulated.
- Nothing here has yet been validated against wind-tunnel or flight data. Until it has,
  treat results as engineering estimates, not guarantees.

## Roadmap

**1. Fidelity (makes the predictions bankable)**
- Airfoil database from a viscous panel solver across Reynolds number and flap deflection;
  custom coordinate import; reflexed sections for flying wings.
- Motor, propeller, ESC and battery catalogue with matched propeller performance maps.
- Validation suite: published wind-tunnel cases and instrumented flights of reference
  airframes, with accuracy reported per quantity inside the product.
- Structural sizing (spar, skin) so mass comes from loads, not a fraction.

**2. Simulation and test**
- Nonlinear six-degree-of-freedom simulation from an aerodynamic database, with wind and gusts.
- VTOL transition corridor and hover control allocation.
- ArduPilot and PX4: parameter file export, software-in-the-loop model export, starting gains.
- Mission simulation with a full energy budget.

**3. Design breadth**
- T-tail, twin-boom, inverted V, canard, tandem wing, tilt-rotor and tailsitter layouts.
- Flaps and multi-segment wings; optimisation ("maximise endurance subject to these checks").
- Parametric sweeps and side-by-side comparison of design variants.
- Units switch, richer CAD output (control-surface cuts, ribs and spars, drawings, PDF report).

**4. Product**
- Accounts, cloud projects, version history, team sharing.
- Subscription billing (monthly and yearly) with tiers, for example: Free (one project,
  no STEP export), Pro (everything for one engineer), Team (shared projects, API, priority compute).
- Analysis jobs moved to a worker queue so heavy studies do not block the interface.

## Licensing notes for a commercial product

- MachUpX and airfoil_db are MIT licensed, which permits commercial use provided the
  copyright notice is retained. "MachUp" is USU's name: do not use it in the product name
  or imply endorsement.
- three.js is MIT. OpenCASCADE (via `cadquery-ocp`) is LGPL 2.1 with an exception: it may be
  used in a closed-source service, but changes to OpenCASCADE itself must be shared.
- Have counsel confirm before launch; this is an engineering summary, not legal advice.
