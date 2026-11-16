"""Data model: what the user asks for (Requirements) and the aircraft that results (Design).

Conventions (same as MachUpX): SI units, body-fixed axes with x forward, y out the
right wing, z down. The datum (0, 0, 0) is the quarter-chord point of the main-wing root.
"""

from typing import Dict, List, Literal, Optional

from pydantic import BaseModel, Field

Config = Literal["conventional", "flying_wing", "vtol_quadplane"]
TailType = Literal["conventional", "v_tail"]


class Requirements(BaseModel):
    """The handful of numbers an engineer knows before the aircraft exists."""

    name: str = "My UAV"
    config: Config = "conventional"

    # Mission
    payload_kg: float = Field(1.0, ge=0, le=100)
    cruise_speed_ms: float = Field(18.0, gt=5, le=80)
    stall_speed_ms: float = Field(11.0, gt=3, le=50)
    endurance_min: float = Field(60.0, gt=1, le=1440)
    cruise_altitude_m: float = Field(100.0, ge=0, le=6000)
    hover_time_min: float = Field(3.0, ge=0, le=30)  # VTOL only: total hover per flight
    range_km: Optional[float] = Field(None, gt=0, le=5000)  # optional target, checked if given
    tail: TailType = "conventional"  # ignored for a flying wing

    # Shape preferences
    aspect_ratio: float = Field(8.0, ge=3, le=20)
    taper_ratio: float = Field(0.6, ge=0.2, le=1.0)
    wing_airfoil: str = "2412"
    static_margin: float = Field(0.12, ge=0.02, le=0.4)

    # Technology assumptions
    battery_wh_per_kg: float = Field(180.0, gt=50, le=400)
    battery_usable_fraction: float = Field(0.8, gt=0.3, le=1.0)
    structure_fraction: float = Field(0.30, gt=0.1, le=0.6)
    avionics_kg: float = Field(0.25, ge=0, le=20)
    avionics_power_w: float = Field(8.0, ge=0, le=500)
    prop_efficiency: float = Field(0.65, gt=0.2, le=0.9)
    drive_efficiency: float = Field(0.85, gt=0.3, le=0.98)  # motor x ESC
    climb_rate_ms: float = Field(3.0, ge=0.5, le=15)
    rotor_disk_loading_nm2: float = Field(120.0, gt=20, le=600)  # VTOL only


class Surface(BaseModel):
    """A lifting surface, described the way MachUpX describes one."""

    span: float  # tip to tip for mirrored surfaces, root to tip for a fin
    root_chord: float
    tip_chord: float
    sweep_deg: float = 0.0  # quarter-chord sweep, positive aft
    dihedral_deg: float = 0.0
    incidence_deg: float = 0.0  # root mounting angle
    washout_deg: float = 0.0  # tip twist relative to root, negative = nose down
    airfoil: str = "2412"
    x: float = 0.0  # root quarter-chord position, positive forward
    z: float = 0.0  # positive down
    control_chord_fraction: float = 0.25
    control_span_start: float = 0.0
    control_span_end: float = 1.0

    @property
    def area(self) -> float:
        return 0.5 * (self.root_chord + self.tip_chord) * self.span

    @property
    def taper(self) -> float:
        return self.tip_chord / self.root_chord

    @property
    def mac(self) -> float:
        t = self.taper
        return (2.0 / 3.0) * self.root_chord * (1 + t + t * t) / (1 + t)


class Fuselage(BaseModel):
    length: float
    width: float
    height: float
    nose_x: float  # x of the nose tip (positive, ahead of the datum)
    z: float = 0.0  # centreline, positive down


class LiftRotors(BaseModel):
    count: int = 4
    diameter: float
    boom_y: float  # spanwise boom position
    front_x: float
    rear_x: float
    boom_diameter: float
    z: float = 0.0


class MassItem(BaseModel):
    name: str
    mass_kg: float
    x: float
    y: float = 0.0
    z: float = 0.0
    note: str = ""


class Propulsion(BaseModel):
    layout: Literal["tractor", "pusher"] = "tractor"
    prop_diameter: float
    motor_power_w: float  # maximum continuous shaft power
    prop_x: float
    hover_power_w: float = 0.0  # electrical, total, at hover
    lift_motor_power_w: float = 0.0  # per lift motor, peak shaft


class Design(BaseModel):
    """A complete parametric aircraft. Sizing creates one; the user may then edit it."""

    requirements: Requirements
    mass_kg: float
    wing: Surface
    tail_type: TailType = "conventional"
    htail: Optional[Surface] = None  # for a V-tail: the two panels, with their dihedral
    vtail: Optional[Surface] = None  # one fin; winglets on a flying wing are two
    fuselage: Fuselage
    propulsion: Propulsion
    lift_rotors: Optional[LiftRotors] = None
    masses: List[MassItem] = []
    battery_wh: float = 0.0
    cg_x: float = 0.0
    cg_z: float = 0.0
    inertia: Dict[str, float] = {}  # Ixx, Iyy, Izz in kg m^2
    body_cd_override: Optional[float] = None  # user-supplied drag coefficient of all non-lifting parts
    source: Literal["requirements", "spreadsheet"] = "requirements"
    sheet_inputs: Optional[Dict] = None  # the spreadsheet inputs this design was built from
    notes: List[str] = []
