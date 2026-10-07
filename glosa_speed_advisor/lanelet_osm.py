#!/usr/bin/env python3
"""
Reads the traffic lights of a lanelet2 map (.osm) without any ROS or lanelet2
dependency, so GLOSA can take stop lines from the map instead of from YAML.

For every traffic light regulatory element and every lanelet that refers to it,
this gives the point where the lanelet centreline meets the stop line, the
driving direction there and the lanelet speed limit.

The map nodes must carry local_x / local_y tags (map-frame coordinates).

List the traffic lights of a map (no build needed):
    python3 lanelet_osm.py /path/to/lanelet2_map.osm
    python3 lanelet_osm.py /path/to/lanelet2_map.osm --id 1234
    python3 lanelet_osm.py /path/to/lanelet2_map.osm --near 1548.9 893.7
"""
import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

Point = Tuple[float, float]

LOOKBACK_M = 5.0          # length used to measure the driving direction
JOIN_TOLERANCE_M = 1.0    # two lanelets are connected if their ends are this close
MPH = 1.609344


class MapError(ValueError):
    pass


@dataclass
class MapStopLine:
    traffic_light_id: int           # ID of the traffic light regulatory element
    lanelet_id: int                 # lanelet that refers to it
    x: float                        # centreline x stop line, map frame [m]
    y: float
    heading_rad: float              # driving direction towards the stop line
    speed_limit_kmh: Optional[float]


# ----------------------------------------------------------------- geometry

def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _cumlen(pts: List[Point]) -> List[float]:
    out = [0.0]
    for i in range(1, len(pts)):
        out.append(out[-1] + _dist(pts[i - 1], pts[i]))
    return out


def _point_at(pts: List[Point], cum: List[float], s: float) -> Point:
    """Point at arc length s along the polyline."""
    s = min(max(s, 0.0), cum[-1])
    for i in range(1, len(pts)):
        if s <= cum[i] or i == len(pts) - 1:
            seg = cum[i] - cum[i - 1]
            f = 0.0 if seg <= 0.0 else (s - cum[i - 1]) / seg
            return (pts[i - 1][0] + f * (pts[i][0] - pts[i - 1][0]),
                    pts[i - 1][1] + f * (pts[i][1] - pts[i - 1][1]))
    return pts[-1]


def _resample(pts: List[Point], n: int) -> List[Point]:
    cum = _cumlen(pts)
    if cum[-1] <= 0.0:
        return [pts[0]] * n
    return [_point_at(pts, cum, cum[-1] * i / (n - 1)) for i in range(n)]


def _cross(a: Point, b: Point) -> float:
    return a[0] * b[1] - a[1] * b[0]


def centerline(left: List[Point], right: List[Point]) -> List[Point]:
    """Centreline of a lanelet, in driving direction (left bound on the left)."""
    # make both bounds run the same way
    if (_dist(left[0], right[0]) + _dist(left[-1], right[-1])
            > _dist(left[0], right[-1]) + _dist(left[-1], right[0])):
        right = right[::-1]
    length = max(_cumlen(left)[-1], _cumlen(right)[-1])
    n = max(len(left), len(right), int(math.ceil(length)) + 1, 2)
    lft, rgt = _resample(left, n), _resample(right, n)
    mid = [((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0) for a, b in zip(lft, rgt)]
    # the left bound must be on the left of the driving direction
    side = sum(_cross((mid[i + 1][0] - mid[i][0], mid[i + 1][1] - mid[i][1]),
                      (lft[i][0] - rgt[i][0], lft[i][1] - rgt[i][1]))
               for i in range(n - 1))
    return mid[::-1] if side < 0.0 else mid


def _segment_hit(p: Point, p2: Point, q: Point, q2: Point) -> Optional[float]:
    """Fraction along p->p2 where it crosses q->q2, or None."""
    r = (p2[0] - p[0], p2[1] - p[1])
    s = (q2[0] - q[0], q2[1] - q[1])
    den = _cross(r, s)
    if abs(den) < 1e-12:
        return None
    qp = (q[0] - p[0], q[1] - p[1])
    t, u = _cross(qp, s) / den, _cross(qp, r) / den
    return t if -1e-9 <= t <= 1.0 + 1e-9 and -1e-9 <= u <= 1.0 + 1e-9 else None


def _dist_to_polyline(p: Point, line: List[Point]) -> float:
    best = math.inf
    for i in range(1, len(line)):
        a, b = line[i - 1], line[i]
        ab = (b[0] - a[0], b[1] - a[1])
        den = ab[0] * ab[0] + ab[1] * ab[1]
        f = 0.0 if den <= 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / den))
        best = min(best, _dist(p, (a[0] + f * ab[0], a[1] + f * ab[1])))
    return best


def stop_arclength(center: List[Point], stop_line: Optional[List[Point]]) -> float:
    """Arc length along the centreline where it meets the stop line."""
    cum = _cumlen(center)
    if not stop_line or len(stop_line) < 2:
        return cum[-1]                       # no stop line: the end of the lanelet
    for i in range(1, len(center)):
        for j in range(1, len(stop_line)):
            t = _segment_hit(center[i - 1], center[i], stop_line[j - 1], stop_line[j])
            if t is not None:
                return cum[i - 1] + t * (cum[i] - cum[i - 1])
    # They only touch (lanelet starts or ends at the stop line): closest point.
    steps = max(int(cum[-1] / 0.25), 1)
    cands = [cum[-1] * k / steps for k in range(steps + 1)]
    return min(cands, key=lambda s: _dist_to_polyline(_point_at(center, cum, s), stop_line))


def _heading_before(center: List[Point], s: float) -> Optional[float]:
    """Driving direction over the last LOOKBACK_M before arc length s."""
    cum = _cumlen(center)
    s0 = max(s - LOOKBACK_M, 0.0)
    if s - s0 < 0.5:
        return None
    a, b = _point_at(center, cum, s0), _point_at(center, cum, s)
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _heading_after(center: List[Point], s: float) -> Optional[float]:
    cum = _cumlen(center)
    s1 = min(s + 2.0, cum[-1])
    if s1 - s < 0.2:
        return None
    a, b = _point_at(center, cum, s), _point_at(center, cum, s1)
    return math.atan2(b[1] - a[1], b[0] - a[0])


def parse_speed_limit(text: Optional[str]) -> Optional[float]:
    """'30', '30.0', '30 km/h', '20 mph' -> km/h."""
    if not text:
        return None
    m = re.match(r"\s*([0-9]+(?:\.[0-9]+)?)\s*([a-zA-Z/]*)", text)
    if not m:
        return None
    v = float(m.group(1))
    return v * MPH if m.group(2).lower() == "mph" else v


# ------------------------------------------------------------------ parsing

def load_traffic_lights(path: str) -> Dict[int, List[MapStopLine]]:
    """traffic light regulatory element ID -> its stop points (one per lane)."""
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as e:
        raise MapError(f"cannot read map '{path}': {e}")

    def tags(el):
        return {t.get("k"): t.get("v") for t in el.findall("tag")}

    nodes: Dict[str, Point] = {}
    without_local = 0
    for n in root.findall("node"):
        t = tags(n)
        if "local_x" in t and "local_y" in t:
            nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
        else:
            without_local += 1
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}

    def way_points(ref: str) -> List[Point]:
        if ref not in ways:
            raise MapError(f"way {ref} is missing in the map")
        try:
            return [nodes[r] for r in ways[ref]]
        except KeyError:
            raise MapError(
                "the map has nodes without local_x / local_y tags "
                f"({without_local} of {without_local + len(nodes)}), so map-frame "
                "coordinates cannot be read from the file")

    lanelets = {}        # id -> (left ref, right ref, [regulatory element refs], speed)
    lights = {}          # id -> [ref_line way refs]
    for rel in root.findall("relation"):
        t = tags(rel)
        members = [(m.get("type"), m.get("ref"), m.get("role")) for m in rel.findall("member")]
        if t.get("type") == "lanelet":
            left = [r for ty, r, role in members if ty == "way" and role == "left"]
            right = [r for ty, r, role in members if ty == "way" and role == "right"]
            if left and right:
                regs = [r for ty, r, role in members if ty == "relation" and role == "regulatory_element"]
                lanelets[rel.get("id")] = (left[0], right[0], regs, parse_speed_limit(t.get("speed_limit")))
        elif t.get("type") == "regulatory_element" and t.get("subtype") == "traffic_light":
            lights[rel.get("id")] = [r for ty, r, role in members if ty == "way" and role == "ref_line"]

    centers: Dict[str, List[Point]] = {}

    def center_of(lid: str) -> List[Point]:
        if lid not in centers:
            left, right, _, _ = lanelets[lid]
            centers[lid] = centerline(way_points(left), way_points(right))
        return centers[lid]

    def predecessor(lid: str) -> Optional[str]:
        """Lanelet that ends where this one starts (the road before the stop line)."""
        start = center_of(lid)[0]
        best, best_d = None, JOIN_TOLERANCE_M
        for other in lanelets:
            if other == lid:
                continue
            try:
                d = _dist(center_of(other)[-1], start)
            except MapError:
                continue
            if d <= best_d:
                best, best_d = other, d
        return best

    out: Dict[int, List[MapStopLine]] = {}
    for lid, (_, _, regs, speed) in lanelets.items():
        for reg in regs:
            if reg not in lights:
                continue
            center = center_of(lid)
            stop_line = None
            for ref in lights[reg]:
                pts = way_points(ref)
                stop_line = pts if stop_line is None else stop_line + pts
            s = stop_arclength(center, stop_line)
            x, y = _point_at(center, _cumlen(center), s)
            heading = _heading_before(center, s)
            if s < 2.0:
                # The lanelet starts at the stop line (lanelet inside the intersection):
                # direction and speed limit come from the lanelet before it.
                prev = predecessor(lid)
                if prev is not None:
                    pc = center_of(prev)
                    heading = _heading_before(pc, _cumlen(pc)[-1]) or heading
                    speed = lanelets[prev][3] or speed
            if heading is None:
                heading = _heading_after(center, s)
            if heading is None:
                continue
            stops = out.setdefault(int(reg), [])
            if any(_dist((x, y), (o.x, o.y)) < JOIN_TOLERANCE_M for o in stops):
                continue                     # same lane seen through another lanelet
            stops.append(MapStopLine(int(reg), int(lid), x, y, heading, speed))
    return out


# ---------------------------------------------------------------------- CLI

def main(argv=None):
    p = argparse.ArgumentParser(description="List the traffic lights of a lanelet2 map")
    p.add_argument("map", help="path to lanelet2_map.osm")
    p.add_argument("--id", type=int, help="show only this traffic light")
    p.add_argument("--near", type=float, nargs=2, metavar=("X", "Y"),
                   help="sort by distance to this map-frame point")
    a = p.parse_args(argv)
    try:
        lights = load_traffic_lights(a.map)
    except MapError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    rows = [s for stops in lights.values() for s in stops if a.id is None or s.traffic_light_id == a.id]
    if a.near:
        rows.sort(key=lambda s: _dist((s.x, s.y), tuple(a.near)))
    else:
        rows.sort(key=lambda s: (s.traffic_light_id, s.lanelet_id))
    print(f"{'traffic_light':>13} {'lanelet':>9} {'stop_x':>10} {'stop_y':>10} {'heading':>8} {'limit':>7}"
          + ("   dist" if a.near else ""))
    for s in rows:
        limit = "?" if s.speed_limit_kmh is None else f"{s.speed_limit_kmh:.0f}"
        line = (f"{s.traffic_light_id:>13} {s.lanelet_id:>9} {s.x:>10.2f} {s.y:>10.2f} "
                f"{math.degrees(s.heading_rad):>7.1f}° {limit:>4} km/h")
        if a.near:
            line += f" {_dist((s.x, s.y), tuple(a.near)):>6.0f} m"
        print(line)
    print(f"{len(lights)} traffic light(s), {len(rows)} stop point(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
