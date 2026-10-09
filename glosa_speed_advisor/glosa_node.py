#!/usr/bin/env python3
"""
GLOSA speed advisor for Autoware (fixed-time signals).

Inputs
    /v2i/spat/raw                   v2i_spat_msgs/SpatPacket   (signal state + end time)
    /localization/kinematic_state   nav_msgs/Odometry          (pose, heading, speed)

Outputs
    /planning/scenario_planning/max_velocity_candidates   VelocityLimit (sender "glosa")
    /planning/scenario_planning/clear_velocity_limit      VelocityLimitClearCommand
    ~/status                                              std_msgs/String (for debugging)

Autoware applies min(map speed limit, this limit, ...), so the map limit is
never exceeded and all stop decisions (red light, obstacles) stay in Autoware.

The decision is computed whenever a SPaT message for a configured approach
arrives. A slow watchdog timer clears the limit if SPaT or the pose goes stale.

Topic names are launch arguments (launch/glosa_speed_advisor.launch.py). Every
other setting comes from one parameter file (config/glosa.param.yaml) and can
also be changed while the node runs:  ros2 param set /glosa_speed_advisor <name> <value>
"""
import math
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from autoware_internal_planning_msgs.msg import (VelocityLimit,
                                                 VelocityLimitClearCommand)
from v2i_spat_msgs.msg import MovementEvent, SpatPacket

from glosa_speed_advisor import lanelet_osm
from glosa_speed_advisor.glosa_core import (GREEN, KMH, RED, UNKNOWN, YELLOW,
                                            Approach, GlosaEngine, Params,
                                            Phase, SignalTiming, v_min_for,
                                            now_tenths_of_hour, remaining_s)

# name -> default. topics.* and map.osm_file are set by the launch file, the rest by
# config/glosa.param.yaml (keep the defaults in sync with both).
DEFAULTS: Dict[str, Any] = {
    # read once at startup
    "topics.spat": "/v2i/spat/raw",
    "topics.odometry": "/localization/kinematic_state",
    "topics.velocity_limit": "/planning/scenario_planning/max_velocity_candidates",
    "topics.clear_velocity_limit": "/planning/scenario_planning/clear_velocity_limit",
    "topics.status": "~/status",
    "map.osm_file": "",
    "output.sender": "glosa",
    "output.clear_on_startup": True,
    "timing.watchdog_rate_hz": 2.0,
    # can be changed at runtime
    "output.republish_period_s": 1.0,
    "output.use_constraints": False,
    "output.max_acceleration": 1.0,
    "output.min_acceleration": -1.0,
    "output.max_jerk": 1.0,
    "output.min_jerk": -1.0,
    "timing.spat_timeout_s": 1.0,
    "timing.odometry_timeout_s": 1.0,
    "timing.min_hold_s": 1.0,
    "vehicle.base_link_to_front_m": 4.05,
    "activation.distance_m": 300.0,
    "activation.min_speed_kmh": 10.0,
    "activation.lateral_tolerance_m": 6.0,
    "activation.heading_tolerance_deg": 45.0,
    "activation.require_fixed_time": True,
    "activation.fixed_time_tolerance_s": 0.3,
    "rules.v_min_ratio": 0.5,
    "rules.v_min_kmh": 10.0,
    "rules.margin_green_start_s": 1.0,
    "rules.margin_green_end_s": 2.0,
    "rules.freeze_distance_m": 30.0,
    "rules.yellow_ignore_distance_m": 50.0,
    "rules.step_kmh": 5.0,
    "rules.raise_hysteresis_kmh": 1.0,
    "rules.use_speed_correction": True,
    "rules.accel_for_correction_mps2": 1.0,
    "approaches.names": [""],
}

# parameter name -> field of glosa_core.Params
PARAM_FIELDS = {
    "vehicle.base_link_to_front_m": "base_link_to_front_m",
    "activation.distance_m": "activation_distance_m",
    "activation.min_speed_kmh": "min_activation_speed_kmh",
    "activation.lateral_tolerance_m": "lateral_tolerance_m",
    "activation.heading_tolerance_deg": "heading_tolerance_deg",
    "activation.require_fixed_time": "require_fixed_time",
    "activation.fixed_time_tolerance_s": "fixed_time_tolerance_s",
    "rules.v_min_ratio": "v_min_ratio",
    "rules.v_min_kmh": "v_min_kmh",
    "rules.margin_green_start_s": "margin_green_start_s",
    "rules.margin_green_end_s": "margin_green_end_s",
    "rules.freeze_distance_m": "freeze_distance_m",
    "rules.yellow_ignore_distance_m": "yellow_ignore_distance_m",
    "rules.step_kmh": "step_kmh",
    "rules.raise_hysteresis_kmh": "raise_hysteresis_kmh",
    "rules.use_speed_correction": "use_speed_correction",
    "rules.accel_for_correction_mps2": "accel_for_correction_mps2",
    "timing.spat_timeout_s": "spat_timeout_s",
    "timing.odometry_timeout_s": "odometry_timeout_s",
    "timing.min_hold_s": "min_hold_s",
}

# per-approach fields: name -> (required, default)
APPROACH_FIELDS = {
    "intersection_id": (True, None),
    "signal_group": (True, None),
    "traffic_light_group_id": (False, 0),   # > 0: stop line, heading, speed limit from the map
    "stop_line_x": (False, 0.0),            # the next three are used when the ID above is 0
    "stop_line_y": (False, 0.0),
    "heading_deg": (False, 0.0),
    "speed_limit_kmh": (False, 0.0),        # 0 = from the map (needs traffic_light_group_id)
    "green_s": (False, 0.0),
    "yellow_s": (False, 0.0),
    "red_s": (False, 0.0),
}

OUTPUT_FLOATS = ("output.republish_period_s", "output.max_acceleration",
                 "output.min_acceleration", "output.max_jerk", "output.min_jerk")


def _event_state_table() -> Dict[int, str]:
    """MovementEvent enum -> RED/GREEN/YELLOW (only constants that exist)."""
    names = {
        "STOP_AND_REMAIN": RED,
        "STOP_THEN_PROCEED": RED,
        "PROTECTED_MOVEMENT_ALLOWED": GREEN,
        "PERMISSIVE_MOVEMENT_ALLOWED": GREEN,
        "PROTECTED_CLEARANCE": YELLOW,
        "PERMISSIVE_CLEARANCE": YELLOW,
    }
    table = {}
    for name, state in names.items():
        if hasattr(MovementEvent, name):
            table[getattr(MovementEvent, name)] = state
    return table


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _as_bool(name: str, v: Any) -> bool:
    if isinstance(v, bool):
        return v
    raise ValueError(f"{name} must be true or false")


def _as_float(name: str, v: Any) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{name} must be a number")
    return float(v)


class GlosaNode(Node):
    def __init__(self):
        super().__init__("glosa_speed_advisor")

        # Dynamic typing: "15" and "15.0" are both accepted in the YAML file.
        dyn = ParameterDescriptor(dynamic_typing=True)
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default, dyn)
        self.approach_names: List[str] = [
            n for n in (self.get_parameter("approaches.names").value or []) if n]
        for n in self.approach_names:
            for fld, (_, default) in APPROACH_FIELDS.items():
                self.declare_parameter(f"approaches.{n}.{fld}", default, dyn)

        # Lanelet2 map (optional): source of the stop lines for approaches that
        # give a traffic_light_group_id. Read once at startup.
        self.map_file = str(self.get_parameter("map.osm_file").value or "")
        self.map_lights = None
        if self.map_file:
            self.map_lights = lanelet_osm.load_traffic_lights(self.map_file)
            self.get_logger().info(
                f"map: {len(self.map_lights)} traffic light(s) read from {self.map_file}")

        params, approaches, self.out = self._load({})
        self.engine = GlosaEngine(params, approaches)
        self._reload_pending = False
        self.add_on_set_parameters_callback(self._on_set_parameters)

        gp = lambda n: self.get_parameter(n).value
        self.sender = str(gp("output.sender"))
        self.event_table = _event_state_table()
        self.last_publish_t = 0.0

        spat_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        limit_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_limit = self.create_publisher(VelocityLimit, gp("topics.velocity_limit"), limit_qos)
        self.pub_clear = self.create_publisher(
            VelocityLimitClearCommand, gp("topics.clear_velocity_limit"), limit_qos)
        self.pub_status = self.create_publisher(String, gp("topics.status"), 10)
        self.create_subscription(SpatPacket, gp("topics.spat"), self.on_spat, spat_qos)
        self.create_subscription(Odometry, gp("topics.odometry"), self.on_odom, 10)
        self.create_timer(1.0 / _as_float("timing.watchdog_rate_hz", gp("timing.watchdog_rate_hz")),
                          self.on_watchdog)

        if gp("output.clear_on_startup"):
            self._publish_clear()  # drop a limit left over from a previous run
        self.get_logger().info(f"GLOSA ready: {len(approaches)} approach(es)")
        for a in approaches:
            self.get_logger().info(
                f"  {a.name}: SPaT {a.intersection_id}/{a.signal_group}, stop line "
                f"({a.stop_x:.1f}, {a.stop_y:.1f}), heading {math.degrees(a.heading_rad):.0f} deg, "
                f"{a.speed_limit_kmh:.0f} km/h, v_min {v_min_for(a.speed_limit_kmh, params):.1f} km/h")

    # --------------------------------------------------------- parameters

    def _load(self, overrides: Dict[str, Any]) -> Tuple[Params, List[Approach], Dict[str, Any]]:
        """Build the configuration from the ROS parameters (+ proposed changes)."""
        def val(name):
            return overrides[name] if name in overrides else self.get_parameter(name).value

        p = Params()
        for pname, fld in PARAM_FIELDS.items():
            conv = _as_bool if isinstance(getattr(p, fld), bool) else _as_float
            setattr(p, fld, conv(pname, val(pname)))
        p.validate()

        approaches = []
        for n in self.approach_names:
            a = {}
            for fld, (required, _) in APPROACH_FIELDS.items():
                pname = f"approaches.{n}.{fld}"
                v = val(pname)
                if v is None:
                    if required:
                        raise ValueError(f"missing parameter {pname}")
                    v = 0.0
                a[fld] = _as_float(pname, v)
            timing = SignalTiming(a["green_s"], a["yellow_s"], a["red_s"])
            ids = (int(a["intersection_id"]), int(a["signal_group"]))
            tl_id = int(a["traffic_light_group_id"])

            if tl_id <= 0:  # stop line typed in by hand
                if a["speed_limit_kmh"] <= 0.0:
                    raise ValueError(f"approaches.{n}.speed_limit_kmh must be > 0")
                approaches.append(Approach(n, ids[0], ids[1], a["stop_line_x"], a["stop_line_y"],
                                           math.radians(a["heading_deg"]), a["speed_limit_kmh"],
                                           timing))
                continue

            # stop line from the lanelet2 map: one entry per lane of that traffic light
            if self.map_lights is None:
                raise ValueError(f"approaches.{n}.traffic_light_group_id is set, but no map "
                                 "is loaded (launch argument map_file)")
            stops = self.map_lights.get(tl_id)
            if not stops:
                raise ValueError(f"approaches.{n}: traffic light {tl_id} is not in the map "
                                 f"(or no lanelet refers to it)")
            for st in stops:
                limit = a["speed_limit_kmh"] if a["speed_limit_kmh"] > 0.0 else st.speed_limit_kmh
                if not limit or limit <= 0.0:
                    raise ValueError(f"approaches.{n}: lanelet {st.lanelet_id} has no speed_limit "
                                     "in the map, set speed_limit_kmh")
                name = n if len(stops) == 1 else f"{n}[lanelet {st.lanelet_id}]"
                approaches.append(Approach(name, ids[0], ids[1], st.x, st.y, st.heading_rad,
                                           float(limit), timing))

        out = {n: _as_float(n, val(n)) for n in OUTPUT_FLOATS}
        out["output.use_constraints"] = _as_bool("output.use_constraints",
                                                 val("output.use_constraints"))
        if out["output.republish_period_s"] < 0.0:
            raise ValueError("output.republish_period_s must be >= 0")
        return p, approaches, out

    def _on_set_parameters(self, changes) -> SetParametersResult:
        """Reject invalid values; valid ones are applied on the next cycle."""
        try:
            self._load({c.name: c.value for c in changes})
        except ValueError as e:
            return SetParametersResult(successful=False, reason=str(e))
        self._reload_pending = True
        return SetParametersResult(successful=True)

    def _apply_pending_parameters(self):
        if not self._reload_pending:
            return
        self._reload_pending = False
        try:
            params, approaches, self.out = self._load({})
            self.engine.configure(params, approaches)
            self.get_logger().info("parameters updated")
        except ValueError as e:
            self.get_logger().error(f"parameter update ignored: {e}")

    # ------------------------------------------------------------- inputs

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.engine.set_ego(p.position.x, p.position.y, _yaw_from_quat(p.orientation),
                            msg.twist.twist.linear.x, time.monotonic())

    def on_spat(self, msg: SpatPacket):
        recv = time.monotonic()
        relevant = False
        for inter in msg.spat.intersections:
            if inter.moy > 0:
                now_t = now_tenths_of_hour(inter.moy, inter.time_stamp)
            else:  # RSU did not send moy: use the UTC wall clock
                utc = datetime.now(timezone.utc)
                now_t = (utc.minute * 60 + utc.second + utc.microsecond / 1e6) * 10.0
            for st in inter.states:
                if not st.events:
                    continue
                ev = st.events[0]  # first event = current state
                state = self.event_table.get(ev.event_state, UNKNOWN)
                t = ev.timing
                rmin = remaining_s(t.min_end_time, now_t) if t.has_min_end_time else None
                rmax = remaining_s(t.max_end_time, now_t) if t.has_max_end_time else None
                relevant |= self.engine.set_phase(inter.intersection_id, st.signal_group,
                                                  Phase(state, rmin, rmax), recv)
        if relevant:
            self._update()  # SPaT-triggered decision

    def on_watchdog(self):
        self._update()

    # ------------------------------------------------------------ outputs

    def _update(self):
        self._apply_pending_parameters()
        now = time.monotonic()
        res = self.engine.step(now)
        if res.changed:
            if res.limit_kmh is None:
                self._publish_clear()
            else:
                self._publish_limit(res.limit_kmh)
            self.get_logger().info(res.status)
        elif res.limit_kmh is not None:
            period = self.out["output.republish_period_s"]
            if period > 0.0 and now - self.last_publish_t >= period:
                self._publish_limit(res.limit_kmh)  # keep the selector in sync
        self.pub_status.publish(String(data=res.status))

    def _publish_limit(self, kmh: float):
        m = VelocityLimit()
        m.stamp = self.get_clock().now().to_msg()
        m.max_velocity = float(kmh * KMH)
        m.sender = self.sender
        if self.out["output.use_constraints"]:
            m.use_constraints = True
            m.constraints.max_acceleration = self.out["output.max_acceleration"]
            m.constraints.min_acceleration = self.out["output.min_acceleration"]
            m.constraints.max_jerk = self.out["output.max_jerk"]
            m.constraints.min_jerk = self.out["output.min_jerk"]
        self.pub_limit.publish(m)
        self.last_publish_t = time.monotonic()

    def _publish_clear(self):
        m = VelocityLimitClearCommand()
        m.stamp = self.get_clock().now().to_msg()
        m.command = True
        m.sender = self.sender
        self.pub_clear.publish(m)

    def shutdown(self):
        """Remove our limit when the node stops (best effort)."""
        if self.engine.limit_kmh is None:
            return
        try:
            self._publish_clear()
            time.sleep(0.2)  # give the message time to leave
        except Exception:  # context already shut down
            pass


def main():
    # Keep the ROS context alive on Ctrl-C so the limit can still be cleared.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = GlosaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
