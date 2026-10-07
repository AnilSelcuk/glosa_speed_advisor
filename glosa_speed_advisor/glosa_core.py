"""
Pure-Python GLOSA logic (no ROS dependency, unit-testable).

Scope
-----
- Fixed-time signals: SPaT minEndTime == maxEndTime, so the remaining time of
  the current phase is exact.
- GLOSA only slows the vehicle down. It never asks for more than the map speed
  limit, and it never makes a stop decision: stopping at red stays in Autoware.

Symbols used in the rules
-------------------------
d        distance from the front bumper to the stop line [m]
v_map    lanelet speed limit [km/h]
v_min    lowest speed GLOSA may ask for [km/h]
T        time until the current signal state ends [s]
G, Y, R  green / yellow / red durations of the signal group [s]
t_map    arrival time at the stop line when driving at v_map [s]

Rules
-----
GREEN   t_map + margin_green_end < T      -> no limit (green is reachable)
        otherwise                         -> target the next green:
                                             T_green = T + Y + R
RED     T_green = T
YELLOW  d >  yellow_ignore_distance       -> T_green = T + R
        d <= yellow_ignore_distance       -> no limit (leave it to Autoware)

With T_green known:
        t_map >= T_green + margin_green_start  -> no limit (map speed is fine)
        otherwise  limit = clamp(d / (T_green + margin_green_start), v_min, v_map)

Always: inside freeze_distance the last decision is kept; stale SPaT, stale
ego pose, unknown state or a passed stop line clear the limit.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

RED, GREEN, YELLOW, UNKNOWN = "RED", "GREEN", "YELLOW", "UNKNOWN"

KMH = 1.0 / 3.6  # km/h -> m/s


# ----------------------------------------------------------------- SPaT time

def now_tenths_of_hour(moy: int, dsecond_ms: int) -> float:
    """Current time as J2735 TimeMark units (0.1 s since the top of the hour)."""
    return (moy % 60) * 600 + dsecond_ms / 100.0


def remaining_s(time_mark: int, now_tenths: float) -> Optional[float]:
    """J2735 TimeMark (0.1 s since top of the hour) -> seconds from now."""
    if time_mark is None or time_mark >= 36000:  # 36000 leap second, 36001 unknown
        return None
    d = time_mark - now_tenths
    if d < -18000:  # wrap-around at the top of the hour
        d += 36000
    elif d > 18000:
        d -= 36000
    return d / 10.0


def time_mark(utc_s: float, remaining: float) -> int:
    """UTC time [s] + remaining [s] -> J2735 TimeMark of the end time."""
    return int(round(((utc_s % 3600.0) + remaining) * 10.0)) % 36000


def minute_of_year(utc_s: float) -> int:
    """UTC time [s since epoch] -> J2735 MinuteOfTheYear."""
    import calendar
    import time as _time
    year_start = calendar.timegm((_time.gmtime(utc_s).tm_year, 1, 1, 0, 0, 0))
    return int((utc_s - year_start) // 60)


def cycle_state(elapsed_s: float, green_s: float, yellow_s: float,
                red_s: float):
    """Fixed-time cycle green -> yellow -> red. Returns (state, remaining [s])."""
    u = elapsed_s % (green_s + yellow_s + red_s)
    if u < green_s:
        return GREEN, green_s - u
    if u < green_s + yellow_s:
        return YELLOW, green_s + yellow_s - u
    return RED, green_s + yellow_s + red_s - u


# -------------------------------------------------------------------- types

@dataclass
class Phase:
    state: str                 # RED / GREEN / YELLOW / UNKNOWN
    rem_min: Optional[float]   # seconds until the earliest possible end
    rem_max: Optional[float]   # seconds until the latest possible end

    def is_fixed(self, tolerance_s: float = 0.3) -> bool:
        """True when SPaT gives an exact end time (fixed-time behaviour)."""
        if self.rem_min is None:
            return False
        if self.rem_max is None:
            return True
        return abs(self.rem_max - self.rem_min) <= tolerance_s

    def aged(self, dt: float) -> "Phase":
        """Same phase, dt seconds later (used between SPaT messages)."""
        f = lambda x: None if x is None else x - dt
        return Phase(self.state, f(self.rem_min), f(self.rem_max))

    def time_to_change(self) -> Optional[float]:
        """Remaining time used by the rules.

        Fixed-time: min == max. If they differ (adaptive signal), red is
        planned against max and green/yellow against min (the safe side).
        """
        if self.state == RED and self.rem_max is not None:
            t = self.rem_max
        elif self.rem_min is not None:
            t = self.rem_min
        else:
            t = self.rem_max
        return None if t is None else max(t, 0.0)


@dataclass
class SignalTiming:
    """Fixed-time plan of one signal group [s]. None or <= 0 means unknown."""
    green_s: Optional[float] = None
    yellow_s: Optional[float] = None
    red_s: Optional[float] = None

    @staticmethod
    def _known(x: Optional[float]) -> bool:
        return x is not None and x > 0.0

    @property
    def G(self) -> Optional[float]:
        return self.green_s if self._known(self.green_s) else None

    @property
    def Y(self) -> Optional[float]:
        return self.yellow_s if self._known(self.yellow_s) else None

    @property
    def R(self) -> Optional[float]:
        return self.red_s if self._known(self.red_s) else None


@dataclass
class Approach:
    """One signalized approach: a stop line, its signal group and speed limit."""
    name: str
    intersection_id: int
    signal_group: int
    stop_x: float               # stop line centre, map frame [m]
    stop_y: float
    heading_rad: float          # driving direction towards the stop line
    speed_limit_kmh: float      # same as the lanelet speed limit in the map
    timing: SignalTiming = field(default_factory=SignalTiming)

    @property
    def key(self) -> Tuple[int, int]:
        return (self.intersection_id, self.signal_group)

    def project(self, x: float, y: float) -> Tuple[float, float]:
        """(distance to stop line along the approach heading, lateral offset)."""
        dx, dy = self.stop_x - x, self.stop_y - y
        c, s = math.cos(self.heading_rad), math.sin(self.heading_rad)
        return dx * c + dy * s, -dx * s + dy * c


@dataclass
class Params:
    # --- vehicle
    base_link_to_front_m: float = 4.05   # wheel base + front overhang
    # --- when GLOSA is allowed to act
    activation_distance_m: float = 300.0
    min_activation_speed_kmh: float = 10.0
    lateral_tolerance_m: float = 6.0
    heading_tolerance_deg: float = 45.0
    require_fixed_time: bool = True
    fixed_time_tolerance_s: float = 0.3
    # --- rule set
    v_min_kmh: float = 15.0
    margin_green_start_s: float = 1.0
    margin_green_end_s: float = 2.0
    freeze_distance_m: float = 30.0
    yellow_ignore_distance_m: float = 50.0
    step_kmh: float = 5.0
    raise_hysteresis_kmh: float = 1.0
    use_speed_correction: bool = True
    accel_for_correction_mps2: float = 1.0
    # --- timing
    spat_timeout_s: float = 1.0
    odometry_timeout_s: float = 1.0
    min_hold_s: float = 1.0

    def validate(self) -> None:
        def positive(name):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be > 0")

        def non_negative(name):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be >= 0")

        for n in ("activation_distance_m", "lateral_tolerance_m", "heading_tolerance_deg",
                  "v_min_kmh", "step_kmh", "accel_for_correction_mps2",
                  "spat_timeout_s", "odometry_timeout_s"):
            positive(n)
        for n in ("base_link_to_front_m", "min_activation_speed_kmh", "fixed_time_tolerance_s",
                  "margin_green_start_s", "margin_green_end_s", "freeze_distance_m",
                  "yellow_ignore_distance_m", "raise_hysteresis_kmh", "min_hold_s"):
            non_negative(n)
        if self.freeze_distance_m >= self.activation_distance_m:
            raise ValueError("freeze_distance_m must be smaller than activation_distance_m")


# -------------------------------------------------------------------- rules

def _angle_diff(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def find_approach(approaches: List[Approach], x: float, y: float, yaw: float,
                  p: Params) -> Optional[Tuple[Approach, float]]:
    """Nearest configured stop line ahead of the ego vehicle, with its distance."""
    hdg_tol = math.radians(p.heading_tolerance_deg)
    best = None
    for a in approaches:
        d, lat = a.project(x, y)
        d -= p.base_link_to_front_m   # the pose is the rear axle; measure from the front bumper
        if (0.0 < d <= p.activation_distance_m and abs(lat) <= p.lateral_tolerance_m
                and _angle_diff(yaw, a.heading_rad) <= hdg_tol):
            if best is None or d < best[1]:
                best = (a, d)
    return best


def arrival_time_s(d: float, v_map_kmh: float, v_cur_kmh: float, p: Params) -> float:
    """Arrival time at the stop line when driving at map speed.

    If the vehicle is currently slower than the map speed, the time lost while
    accelerating back to it is added:  (v_map - v_cur)^2 / (2 * a * v_map).
    """
    v_map = v_map_kmh * KMH
    t = d / v_map
    if p.use_speed_correction:
        dv = v_map - max(v_cur_kmh, 0.0) * KMH
        if dv > 0.0:
            t += dv * dv / (2.0 * p.accel_for_correction_mps2 * v_map)
    return t


@dataclass
class Advice:
    speed_kmh: Optional[float]   # None -> no limit (drive at map speed)
    reason: str


def advise(state: str, T: Optional[float], d: float, v_map_kmh: float,
           v_cur_kmh: float, timing: SignalTiming, p: Params) -> Advice:
    """Rule set. Returns the exact (not yet rounded) speed to ask for."""
    if d <= 0.0:
        return Advice(None, "past stop line")
    if T is None or state == UNKNOWN:
        return Advice(None, "no timing information")
    T = max(T, 0.0)
    t_map = arrival_time_s(d, v_map_kmh, v_cur_kmh, p)
    v_min = min(p.v_min_kmh, v_map_kmh)

    if state == GREEN:
        if t_map + p.margin_green_end_s < T:
            return Advice(None, "green reachable at map speed")
        if timing.Y is None or timing.R is None:
            return Advice(None, "green not reachable, yellow/red duration unknown")
        t_green = T + timing.Y + timing.R
        tag = "green not reachable, next green"
    elif state == YELLOW:
        if d <= p.yellow_ignore_distance_m:
            return Advice(None, "yellow and close to the stop line")
        if timing.R is None:
            return Advice(None, "yellow, red duration unknown")
        t_green = T + timing.R
        tag = "yellow, next green"
    else:  # RED
        t_green = T
        tag = "red"

    t_target = t_green + p.margin_green_start_s
    if t_map >= t_target:
        if timing.G is not None and t_map + p.margin_green_end_s >= t_green + timing.G:
            return Advice(None, f"{tag}: arrival after that green window")
        return Advice(None, f"{tag}: map speed already arrives on green")

    v_req = d / t_target / KMH
    if v_req < v_min:
        return Advice(v_min, f"{tag}: cannot avoid red, minimize waiting")
    return Advice(min(v_req, v_map_kmh), f"{tag}: slow down to arrive on green")


def quantize(raw_kmh: Optional[float], current_kmh: Optional[float],
             v_map_kmh: float, p: Params) -> Optional[float]:
    """Round the requested speed down to step_kmh, with hysteresis when raising.

    The limit is lowered as soon as the rounded value drops, but it is raised
    only when the exact value is raise_hysteresis_kmh above the next step. This
    prevents toggling between two steps.
    """
    if raw_kmh is None:
        return None
    step = p.step_kmh
    q = math.floor(raw_kmh / step + 1e-9) * step
    if current_kmh is not None and q > current_kmh and raw_kmh < q + p.raise_hysteresis_kmh:
        q = max(q - step, current_kmh)
    q = max(q, min(p.v_min_kmh, v_map_kmh))
    if q >= v_map_kmh - 1e-6:
        return None
    return q


# ------------------------------------------------------------------- engine

@dataclass
class Result:
    limit_kmh: Optional[float]   # limit that should be active now (None = none)
    changed: bool                # True when it differs from the previous step
    status: str


class GlosaEngine:
    """Keeps the latest inputs and the active limit. Time is passed in [s]."""

    def __init__(self, params: Params, approaches: List[Approach]):
        self.phases: Dict[Tuple[int, int], Tuple[Phase, float]] = {}
        self.ego: Optional[Tuple[float, float, float, float, float]] = None
        self.limit_kmh: Optional[float] = None
        self.last_change_t = -math.inf
        self.configure(params, approaches)

    def configure(self, params: Params, approaches: List[Approach]) -> None:
        """(Re)load parameters. The active limit and cached inputs are kept."""
        params.validate()
        self.p = params
        self.approaches = list(approaches)
        self.keys = {a.key for a in self.approaches}

    # ---- inputs

    def set_phase(self, intersection_id: int, signal_group: int, phase: Phase, t: float) -> bool:
        """Store a SPaT phase. Returns True if it belongs to a configured approach."""
        key = (int(intersection_id), int(signal_group))
        if key not in self.keys:
            return False
        self.phases[key] = (phase, t)
        return True

    def set_ego(self, x: float, y: float, yaw: float, speed_mps: float, t: float) -> None:
        self.ego = (x, y, yaw, speed_mps, t)

    # ---- decision

    def _target(self, t: float) -> Tuple[Optional[float], str, bool]:
        """(wanted limit, status text, apply immediately)."""
        p = self.p
        if self.ego is None:
            return None, "no ego pose", True
        x, y, yaw, v_mps, t_ego = self.ego
        if t - t_ego > p.odometry_timeout_s:
            return None, f"ego pose timeout ({t - t_ego:.1f}s)", True
        found = find_approach(self.approaches, x, y, yaw, p)
        if found is None:
            return None, "no approach", True
        a, d = found
        entry = self.phases.get(a.key)
        if entry is None:
            return None, f"{a.name}: d={d:.0f}m no SPaT", True
        phase, t_recv = entry
        age = t - t_recv
        if age > p.spat_timeout_s:
            return None, f"{a.name}: d={d:.0f}m SPaT timeout ({age:.1f}s)", True
        phase = phase.aged(age)
        if phase.state == UNKNOWN:
            return None, f"{a.name}: d={d:.0f}m unknown signal state", True
        if p.require_fixed_time and not phase.is_fixed(p.fixed_time_tolerance_s):
            return None, f"{a.name}: d={d:.0f}m min != max (not fixed-time), skipped", True

        T = phase.time_to_change()
        v_kmh = abs(v_mps) / KMH
        head = (f"{a.name}: d={d:.0f}m v={v_kmh:.0f}km/h {phase.state} "
                f"T={'?' if T is None else f'{T:.1f}s'}")
        if d <= p.freeze_distance_m:
            return self.limit_kmh, f"{head} -> frozen (d <= {p.freeze_distance_m:.0f}m)", False

        adv = advise(phase.state, T, d, a.speed_limit_kmh, v_kmh, a.timing, p)
        target = quantize(adv.speed_kmh, self.limit_kmh, a.speed_limit_kmh, p)
        if target is not None and self.limit_kmh is None and v_kmh < p.min_activation_speed_kmh:
            return None, f"{head} -> {adv.reason}, but below activation speed", False
        return target, f"{head} -> {adv.reason}", False

    def step(self, t: float) -> Result:
        target, status, immediate = self._target(t)
        changed = False
        differs = (target is None) != (self.limit_kmh is None) or (
            target is not None and abs(target - self.limit_kmh) > 1e-6)
        if differs and (immediate or t - self.last_change_t >= self.p.min_hold_s):
            self.limit_kmh = target
            self.last_change_t = t
            changed = True
        cur = "none" if self.limit_kmh is None else f"{self.limit_kmh:.0f} km/h"
        return Result(self.limit_kmh, changed, f"[limit: {cur}] {status}")
