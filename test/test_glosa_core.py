import ast
import math
import os

import pytest

from glosa_speed_advisor.glosa_core import (
    GREEN, RED, UNKNOWN, YELLOW, Approach, GlosaEngine, Params, Phase,
    SignalTiming, advise, arrival_time_s, cycle_state, find_approach,
    minute_of_year, now_tenths_of_hour, quantize, remaining_s, time_mark,
    v_min_for)

P = Params()
PLAN = SignalTiming(green_s=40.0, yellow_s=4.0, red_s=46.0)  # 90 s cycle
V = 30.0  # map limit [km/h]


def adv(state, T, d, v_cur=V, timing=PLAN, p=P):
    return advise(state, T, d, V, v_cur, timing, p)


# ------------------------------------------------------------------ SPaT time

def test_remaining_basic_and_wrap():
    now = now_tenths_of_hour(moy=382693, dsecond_ms=46270)   # 18:13:46.27
    assert abs(remaining_s(8317, now) - 5.43) < 0.01
    assert abs(remaining_s(50, 35950) - 10.0) < 1e-6          # across the top of the hour
    assert remaining_s(36001, now) is None                    # "unknown" TimeMark


def test_fixed_detection_and_time_to_change():
    assert Phase(RED, 10.0, 10.1).is_fixed(0.3)
    assert not Phase(RED, 10.0, 30.0).is_fixed(0.3)
    assert Phase(RED, 10.0, 30.0).time_to_change() == 30.0    # red: plan against max
    assert Phase(GREEN, 10.0, 30.0).time_to_change() == 10.0  # green: plan against min


# ---------------------------------------------------------------------- green

def test_green_reachable_no_limit():
    # 100 m at 30 km/h = 12 s, +2 s margin < 20 s
    assert adv(GREEN, 20.0, 100.0).speed_kmh is None


def test_green_margin_boundary():
    # 100 m = 12 s. T = 14.1 -> reachable, T = 13.9 -> not reachable
    assert adv(GREEN, 14.1, 100.0).speed_kmh is None
    assert adv(GREEN, 13.9, 100.0).speed_kmh is not None


def test_green_not_reachable_targets_next_green():
    # 5 s of green left, 250 m away. Next green in 5 + 4 + 46 = 55 s -> 250 / 56 s
    a = adv(GREEN, 5.0, 250.0)
    assert a.speed_kmh == pytest.approx(250.0 / 56.0 * 3.6)
    assert "next green" in a.reason


def test_green_not_reachable_unknown_plan_no_limit():
    assert adv(GREEN, 5.0, 250.0, timing=SignalTiming()).speed_kmh is None


# ------------------------------------------------------------------------ red

def test_red_slow_down_to_hit_green():
    # 250 m, red for 30 s more -> 250 / 31 s = 29.03 km/h
    a = adv(RED, 30.0, 250.0)
    assert a.speed_kmh == pytest.approx(250.0 / 31.0 * 3.6)


def test_red_map_speed_already_fine():
    # 250 m = 30 s at map speed, red ends in 10 s
    assert adv(RED, 10.0, 250.0).speed_kmh is None


def test_red_too_long_minimize_waiting():
    # 100 m, red for 80 s: even v_min arrives early -> v_min
    a = adv(RED, 80.0, 100.0)
    assert a.speed_kmh == v_min_for(V, P) == 15.0
    assert "minimize waiting" in a.reason


def test_red_works_without_plan():
    assert adv(RED, 30.0, 200.0, timing=SignalTiming()).speed_kmh == pytest.approx(
        200.0 / 31.0 * 3.6)


# --------------------------------------------------------------------- yellow

def test_yellow_far_uses_red_rule():
    # 2 s yellow + 46 s red = 48 s -> 200 / 49 s = 14.7 km/h -> below v_min
    assert adv(YELLOW, 2.0, 200.0).speed_kmh == v_min_for(V, P)
    # 300 m / 49 s = 22.0 km/h
    assert adv(YELLOW, 2.0, 300.0).speed_kmh == pytest.approx(300.0 / 49.0 * 3.6)


def test_yellow_close_does_nothing():
    assert adv(YELLOW, 2.0, 50.0).speed_kmh is None
    assert adv(YELLOW, 2.0, 30.0).speed_kmh is None


# ---------------------------------------------------------------- never above

def test_never_above_map_limit_or_below_vmin():
    for state in (RED, GREEN, YELLOW):
        for T in (0.0, 3.0, 10.0, 25.0, 45.0, 80.0):
            for d in (35.0, 60.0, 120.0, 250.0, 300.0):
                s = adv(state, T, d).speed_kmh
                assert s is None or v_min_for(V, P) <= s <= V


def test_speed_correction_adds_acceleration_time():
    # from 10 km/h to 30 km/h at 1 m/s^2: (8.33 - 2.78)^2 / (2 * 8.33) = 1.85 s
    t0 = arrival_time_s(100.0, V, V, P)
    t1 = arrival_time_s(100.0, V, 10.0, P)
    assert t0 == pytest.approx(12.0)
    assert t1 - t0 == pytest.approx(1.852, abs=0.01)
    off = Params(use_speed_correction=False)
    assert arrival_time_s(100.0, V, 10.0, off) == pytest.approx(12.0)


# ------------------------------------------------------------------- rounding

def test_quantize_rounds_down_and_respects_bounds():
    assert quantize(29.0, None, V, P) == 25.0
    assert quantize(23.2, None, V, P) == 20.0
    assert quantize(15.0, None, V, P) == 15.0
    assert quantize(30.0, None, V, P) is None      # equal to the map limit -> no limit
    assert quantize(None, 20.0, V, P) is None


def test_quantize_hysteresis_when_raising():
    assert quantize(25.4, 20.0, V, P) == 20.0      # not far enough above 25
    assert quantize(26.1, 20.0, V, P) == 25.0
    assert quantize(19.9, 25.0, V, P) == 15.0      # lowering is immediate


# --------------------------------------------------------------------- engine

def make_engine(**kw):
    a = Approach("test", 100, 2, 0.0, 0.0, 0.0, V, PLAN)
    kw.setdefault("base_link_to_front_m", 0.0)   # tests give d from the front bumper
    return GlosaEngine(Params(**kw), [a])


def feed(e, t, d, v_kmh, state, T):
    e.set_ego(-d, 0.0, 0.0, v_kmh / 3.6, t)
    e.set_phase(100, 2, Phase(state, T, T), t)
    return e.step(t)


def test_engine_sets_and_clears_limit():
    e = make_engine()
    r = feed(e, 0.0, 250.0, 30.0, RED, 40.0)        # 250 / 41 s = 21.95 -> 20
    assert r.changed and r.limit_kmh == 20.0
    r = feed(e, 5.0, 300.5, 30.0, RED, 40.0)        # outside activation distance
    assert r.changed and r.limit_kmh is None


def test_engine_ignores_other_intersections():
    e = make_engine()
    assert not e.set_phase(999, 2, Phase(RED, 5.0, 5.0), 0.0)
    assert e.set_phase(100, 2, Phase(RED, 5.0, 5.0), 0.0)


def test_engine_spat_timeout_clears():
    e = make_engine()
    assert feed(e, 0.0, 250.0, 30.0, RED, 40.0).limit_kmh == 20.0
    e.set_ego(-240.0, 0.0, 0.0, 6.0, 1.5)           # pose is fresh, SPaT is 1.5 s old
    r = e.step(1.5)
    assert r.changed and r.limit_kmh is None and "SPaT timeout" in r.status


def test_engine_odometry_timeout_clears():
    e = make_engine()
    assert feed(e, 0.0, 250.0, 30.0, RED, 40.0).limit_kmh == 20.0
    e.set_phase(100, 2, Phase(RED, 38.5, 38.5), 1.5)
    r = e.step(1.5)
    assert r.limit_kmh is None and "ego pose timeout" in r.status


def test_engine_freeze_keeps_decision_then_clears_after_stop_line():
    e = make_engine()
    assert feed(e, 0.0, 60.0, 30.0, RED, 20.0).limit_kmh == 15.0
    # inside 30 m the light turns green: the limit is kept
    r = feed(e, 5.0, 25.0, 15.0, GREEN, 40.0)
    assert r.limit_kmh == 15.0 and "frozen" in r.status
    # stop line passed
    r = feed(e, 12.0, -1.0, 15.0, GREEN, 33.0)
    assert r.changed and r.limit_kmh is None


def test_engine_freeze_does_not_start_a_new_limit():
    e = make_engine()
    r = feed(e, 0.0, 25.0, 30.0, RED, 20.0)
    assert r.limit_kmh is None and "frozen" in r.status


def test_engine_activation_speed():
    e = make_engine()
    r = feed(e, 0.0, 200.0, 5.0, RED, 60.0)         # crawling: do not start
    assert r.limit_kmh is None and "activation speed" in r.status
    r = feed(e, 1.0, 199.0, 20.0, RED, 59.0)
    assert r.limit_kmh == 15.0
    r = feed(e, 3.0, 195.0, 5.0, RED, 57.0)         # already active: stays
    assert r.limit_kmh == 15.0


def test_engine_min_hold():
    e = make_engine()
    assert feed(e, 0.0, 250.0, 30.0, RED, 40.0).limit_kmh == 20.0
    r = feed(e, 0.5, 246.0, 30.0, RED, 60.0)        # wants 15, but only 0.5 s passed
    assert not r.changed and r.limit_kmh == 20.0
    r = feed(e, 1.1, 242.0, 30.0, RED, 59.4)
    assert r.changed and r.limit_kmh == 15.0


def test_engine_requires_fixed_time():
    e = make_engine()
    e.set_ego(-250.0, 0.0, 0.0, 8.33, 0.0)
    e.set_phase(100, 2, Phase(RED, 20.0, 45.0), 0.0)
    r = e.step(0.0)
    assert r.limit_kmh is None and "not fixed-time" in r.status
    e2 = make_engine(require_fixed_time=False)
    e2.set_ego(-250.0, 0.0, 0.0, 8.33, 0.0)
    e2.set_phase(100, 2, Phase(RED, 20.0, 45.0), 0.0)
    assert e2.step(0.0).limit_kmh == 15.0           # planned against max: 250 / 46 s


def test_engine_unknown_state_and_wrong_heading():
    e = make_engine()
    assert feed(e, 0.0, 250.0, 30.0, UNKNOWN, 40.0).limit_kmh is None
    e.set_ego(-250.0, 0.0, math.pi, 8.33, 1.0)      # driving away from the stop line
    e.set_phase(100, 2, Phase(RED, 40.0, 40.0), 1.0)
    assert "no approach" in e.step(1.0).status


def test_params_validation():
    with pytest.raises(ValueError):
        Params(step_kmh=0.0).validate()
    with pytest.raises(ValueError):
        Params(freeze_distance_m=400.0).validate()


def test_runtime_reconfigure_keeps_limit_state():
    e = make_engine()
    assert feed(e, 0.0, 250.0, 30.0, RED, 40.0).limit_kmh == 20.0
    e.configure(Params(v_min_kmh=25.0, base_link_to_front_m=0.0), e.approaches)
    r = feed(e, 2.0, 240.0, 20.0, RED, 38.0)        # 240 / 39 s = 22.2 -> v_min 25
    assert r.changed and r.limit_kmh == 25.0


def test_distance_is_measured_from_front_bumper():
    a = Approach("test", 100, 2, 0.0, 0.0, 0.0, V, PLAN)
    _, d = find_approach([a], -100.0, 0.0, 0.0, Params(base_link_to_front_m=4.05))
    assert d == pytest.approx(95.95)
    # front bumper over the stop line -> approach is finished
    assert find_approach([a], -4.0, 0.0, 0.0, Params(base_link_to_front_m=4.05)) is None


# ------------------------------------------------------------- fake SPaT

def test_cycle_state():
    assert cycle_state(0.0, 40.0, 4.0, 46.0) == (GREEN, 40.0)
    assert cycle_state(41.0, 40.0, 4.0, 46.0) == (YELLOW, 3.0)
    assert cycle_state(50.0, 40.0, 4.0, 46.0) == (RED, 40.0)
    assert cycle_state(95.0, 40.0, 4.0, 46.0) == (GREEN, 35.0)   # second cycle


def test_fake_spat_time_round_trip():
    # what the fake publisher writes must decode to the same remaining time
    utc = 1791222466.27                       # some UTC instant
    for rem in (0.4, 12.3, 46.0):
        for shift in (0.0, 3590.0):           # also across the top of the hour
            t = utc + shift
            moy, ms = minute_of_year(t), int((t % 60.0) * 1000)
            got = remaining_s(time_mark(t, rem), now_tenths_of_hour(moy, ms))
            assert got == pytest.approx(rem, abs=0.06)
    assert minute_of_year(1767225600.0) == 0  # 2026-01-01 00:00:00 UTC


# ------------------------------------------------ YAML <-> node consistency

def _literal(tree, name):
    for node in tree.body:
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            target = node.targets[0] if isinstance(node, ast.Assign) else node.target
            if getattr(target, "id", None) == name:
                return ast.literal_eval(node.value)
    raise KeyError(name)


def _flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(_flatten(v, f"{prefix}{k}."))
        else:
            out[f"{prefix}{k}"] = v
    return out


def test_yaml_file_matches_node_parameters():
    yaml = pytest.importorskip("yaml")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(root, "glosa_speed_advisor", "glosa_node.py")) as f:
        tree = ast.parse(f.read())
    defaults = _literal(tree, "DEFAULTS")
    fields = _literal(tree, "APPROACH_FIELDS")
    with open(os.path.join(root, "config", "glosa.param.yaml")) as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]
    names = params["approaches"]["names"]
    in_yaml = _flatten(params)
    approach_keys = {f"approaches.{n}.{k}" for n in names for k in fields}
    topics = {k for k in defaults if k.startswith("topics.") or k == "map.osm_file"}
    # every node parameter except the topics is in the YAML file, and nothing else
    assert set(in_yaml) == (set(defaults) - topics) | approach_keys
    # and the YAML values equal the code defaults
    for k, v in defaults.items():
        if k != "approaches.names" and k not in topics:
            assert in_yaml[k] == v, k
    # the topics are set by the launch file, with the same defaults as the node
    with open(os.path.join(root, "launch", "glosa_speed_advisor.launch.py")) as f:
        launch = f.read()
    for k in topics:
        assert f'"{k}"' in launch, k
        if k not in ("topics.status", "map.osm_file"):   # these differ on purpose
            assert f'default_value="{defaults[k]}"' in launch, k


def test_fake_spat_example_matches_glosa_example():
    """The two example configurations must describe the same signal."""
    yaml = pytest.importorskip("yaml")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(root, "config", "glosa.param.yaml")) as f:
        approaches = yaml.safe_load(f)["/**"]["ros__parameters"]["approaches"]
    with open(os.path.join(root, "config", "fake_spat.param.yaml")) as f:
        fake = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(os.path.join(root, "glosa_speed_advisor", "fake_spat_node.py")) as f:
        tree = ast.parse(f.read())
    fields = _literal(tree, "SIGNAL_FIELDS")
    assert fake["signals"]["names"] == approaches["names"]
    for n in approaches["names"]:
        assert set(fake["signals"][n]) == set(fields)
        for k in ("intersection_id", "signal_group", "green_s", "yellow_s", "red_s"):
            assert fake["signals"][n][k] == approaches[n][k], k


# ------------------------------------------------------------- lanelet2 map

from glosa_speed_advisor.lanelet_osm import (MapError, centerline,  # noqa: E402
                                             load_traffic_lights, parse_speed_limit)


class OsmBuilder:
    """Writes a minimal lanelet2 .osm file."""

    def __init__(self, local=True):
        self.out, self.next_id, self.local = [], 1, local

    def _id(self):
        self.next_id += 1
        return self.next_id

    def way(self, pts, tags=None):
        refs = []
        for x, y in pts:
            i = self._id()
            loc = f'<tag k="local_x" v="{x}"/><tag k="local_y" v="{y}"/>' if self.local else ""
            self.out.append(f'<node id="{i}" lat="0" lon="0">{loc}</node>')
            refs.append(i)
        w = self._id()
        t = "".join(f'<tag k="{k}" v="{v}"/>' for k, v in (tags or {}).items())
        self.out.append(f'<way id="{w}">' + "".join(f'<nd ref="{r}"/>' for r in refs) + t + "</way>")
        return w

    def relation(self, members, tags):
        r = self._id()
        m = "".join(f'<member type="{ty}" ref="{ref}" role="{role}"/>' for ty, ref, role in members)
        t = "".join(f'<tag k="{k}" v="{v}"/>' for k, v in tags.items())
        self.out.append(f'<relation id="{r}">{m}{t}</relation>')
        return r

    def lanelet(self, left, right, speed, regs=()):
        members = [("way", self.way(left), "left"), ("way", self.way(right), "right")]
        members += [("relation", r, "regulatory_element") for r in regs]
        return self.relation(members, {"type": "lanelet", "subtype": "road", "speed_limit": speed})

    def traffic_light(self, stop_line):
        members = [("way", self.way([(0, 0), (0, 1)], {"type": "traffic_light"}), "refers")]
        if stop_line:
            members.append(("way", self.way(stop_line, {"type": "stop_line"}), "ref_line"))
        return self.relation(members, {"type": "regulatory_element", "subtype": "traffic_light"})

    def write(self, path):
        nodes = [o for o in self.out if o.startswith("<node")]
        rest = [o for o in self.out if not o.startswith("<node")]
        path.write_text('<?xml version="1.0"?><osm>' + "".join(nodes + rest) + "</osm>")
        return str(path)


def rot(pts, deg, ox=1000.0, oy=2000.0):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(ox + x * c - y * s, oy + x * s + y * c) for x, y in pts]


def test_map_stop_line_crossing_the_lanelet(tmp_path):
    # 3.5 m wide lanelet along heading 30 deg, stop line across it at 80 m
    b = OsmBuilder()
    tl = b.traffic_light(rot([(80.0, 3.0), (80.0, -3.0)], 30.0))
    lid = b.lanelet(rot([(0, 1.75), (50, 1.75), (100, 1.75)], 30.0),
                    rot([(0, -1.75), (100, -1.75)], 30.0), "30", [tl])
    stops = load_traffic_lights(b.write(tmp_path / "m.osm"))[tl]
    assert len(stops) == 1 and stops[0].lanelet_id == lid
    ex, ey = rot([(80.0, 0.0)], 30.0)[0]
    assert stops[0].x == pytest.approx(ex, abs=0.01) and stops[0].y == pytest.approx(ey, abs=0.01)
    assert math.degrees(stops[0].heading_rad) == pytest.approx(30.0, abs=0.1)
    assert stops[0].speed_limit_kmh == 30.0


def test_map_light_on_the_lanelet_inside_the_intersection(tmp_path):
    # Usual Autoware layout: the traffic light is attached to the lanelets that
    # START at the stop line (straight and left turn). Direction and speed limit
    # must come from the road before the stop line, not from the turn lanelet.
    b = OsmBuilder()
    tl = b.traffic_light([(100.0, 2.0), (100.0, -2.0)])
    b.lanelet([(0, 1.75), (100, 1.75)], [(0, -1.75), (100, -1.75)], "20 mph")       # approach
    b.lanelet([(100, 1.75), (120, 1.75)], [(100, -1.75), (120, -1.75)], "30", [tl])  # straight
    b.lanelet([(100, 1.75), (106, 3.0), (110, 8.0)],
              [(100, -1.75), (109, 0.5), (113.5, 8.0)], "15", [tl])                  # left turn
    stops = load_traffic_lights(b.write(tmp_path / "m.osm"))[tl]
    assert len(stops) == 1                       # both lanelets start at the same point
    assert (stops[0].x, stops[0].y) == pytest.approx((100.0, 0.0), abs=0.05)
    assert math.degrees(stops[0].heading_rad) == pytest.approx(0.0, abs=0.1)
    assert stops[0].speed_limit_kmh == pytest.approx(32.19, abs=0.01)   # 20 mph, from the approach


def test_map_two_lanes_reversed_bounds_and_no_stop_line(tmp_path):
    b = OsmBuilder()
    tl = b.traffic_light([(60.0, 6.0), (60.0, -2.0)])                # spans both lanes
    # lane 1: its right bound is stored in the opposite direction
    b.lanelet([(0, 1.75), (60, 1.75)], [(60, -1.75), (0, -1.75)], "30", [tl])
    # lane 2 (left of lane 1): both bounds stored against the driving direction
    b.lanelet([(60, 1.75), (0, 1.75)], [(60, 5.25), (0, 5.25)], "30", [tl])
    tl2 = b.traffic_light(None)                                       # no ref_line
    b.lanelet([(0, 101.75), (40, 101.75)], [(0, 98.25), (40, 98.25)], "50", [tl2])
    lights = load_traffic_lights(b.write(tmp_path / "m.osm"))
    pts = sorted((round(s.x, 2), round(s.y, 2)) for s in lights[tl])
    assert pts == [(60.0, 0.0), (60.0, 3.5)]
    # lane 1 runs in +x. Lane 2 is stored the other way round, with its left bound
    # at y = 1.75, so it runs in -x and only touches the stop line at its start.
    hd = {round(s.y, 1): round(math.degrees(s.heading_rad)) % 360 for s in lights[tl]}
    assert hd == {0.0: 0, 3.5: 180}
    end = lights[tl2][0]
    assert (end.x, end.y) == pytest.approx((40.0, 100.0), abs=0.01)   # end of the lanelet
    assert end.speed_limit_kmh == 50.0


def test_centerline_direction_follows_the_left_bound():
    c = centerline([(0, 1), (10, 1)], [(0, -1), (10, -1)])
    assert c[0] == pytest.approx((0, 0)) and c[-1] == pytest.approx((10, 0))
    c = centerline([(0, -1), (10, -1)], [(0, 1), (10, 1)])           # left bound is at y = -1
    assert c[0] == pytest.approx((10, 0)) and c[-1] == pytest.approx((0, 0))


def test_map_errors(tmp_path):
    with pytest.raises(MapError):
        load_traffic_lights(str(tmp_path / "missing.osm"))
    b = OsmBuilder(local=False)                                       # no local_x / local_y
    tl = b.traffic_light([(10.0, 2.0), (10.0, -2.0)])
    b.lanelet([(0, 1.75), (20, 1.75)], [(0, -1.75), (20, -1.75)], "30", [tl])
    with pytest.raises(MapError, match="local_x"):
        load_traffic_lights(b.write(tmp_path / "m.osm"))


def test_parse_speed_limit():
    assert parse_speed_limit("30") == 30.0
    assert parse_speed_limit("30.00 km/h") == 30.0
    assert parse_speed_limit("25mph") == pytest.approx(40.23, abs=0.01)
    assert parse_speed_limit(None) is None and parse_speed_limit("fast") is None


def test_v_min_follows_the_lanelet_speed_limit():
    p = Params()                                   # ratio 0.5, floor 10 km/h
    assert v_min_for(35.0, p) == 17.5
    assert v_min_for(20.0, p) == 10.0              # floor
    assert v_min_for(8.0, p) == 8.0                # never above the limit itself
    fixed = Params(v_min_ratio=0.0, v_min_kmh=15.0)
    assert v_min_for(35.0, fixed) == v_min_for(50.0, fixed) == 15.0
    # 35 km/h lane, 100 m, red for 80 s: GLOSA asks for 17.5, not 15
    a = advise(RED, 80.0, 100.0, 35.0, 35.0, PLAN, p)
    assert a.speed_kmh == 17.5
    assert quantize(17.6, None, 35.0, p) == 17.5   # rounding never goes below v_min
    with pytest.raises(ValueError):
        Params(v_min_ratio=0.0, v_min_kmh=0.0).validate()
    with pytest.raises(ValueError):
        Params(v_min_ratio=1.2).validate()
