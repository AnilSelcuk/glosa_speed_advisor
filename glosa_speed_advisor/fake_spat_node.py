#!/usr/bin/env python3
"""
Fake fixed-time SPaT publisher, for testing GLOSA in the planning simulator.

It plays a green -> yellow -> red cycle for each configured signal and publishes
it in the same format as the real V2I bridge (v2i_spat_msgs/SpatPacket, one
packet per intersection, minEndTime == maxEndTime).

Optionally it also publishes the same state as an Autoware traffic signal, so
the simulated vehicle really stops at red (set traffic_light_group_id > 0).

Do NOT run this node on the vehicle together with the real SPaT bridge.

Restart all cycles from the beginning (handy for repeatable test runs):
    ros2 service call /fake_spat/restart std_srvs/srv/Trigger
"""
import time
from typing import Any, Dict, List, Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_srvs.srv import Trigger

from v2i_spat_msgs.msg import (IntersectionState, MovementEvent, MovementState,
                               Spat, SpatPacket, TimeChangeDetails)

from glosa_speed_advisor.glosa_core import (GREEN, RED, YELLOW, cycle_state,
                                            minute_of_year, time_mark)

try:  # optional: only needed when traffic_light_group_id is used
    from autoware_perception_msgs.msg import (TrafficLightElement,
                                              TrafficLightGroup,
                                              TrafficLightGroupArray)
except ImportError:  # other Autoware version: SPaT only
    TrafficLightGroupArray = None

# name -> default. topics.* are set by the launch file, the rest by
# config/fake_spat.param.yaml.
DEFAULTS: Dict[str, Any] = {
    "topics.spat": "/v2i/spat/raw",
    "topics.traffic_signals": "/perception/traffic_light_recognition/traffic_signals",
    "rate_hz": 10.0,
    "signals.names": [""],
}

# per-signal fields: name -> default (None = required)
SIGNAL_FIELDS = {
    "intersection_id": None,
    "signal_group": None,
    "green_s": None,
    "yellow_s": None,
    "red_s": None,
    "offset_s": 0.0,
    "traffic_light_group_id": 0,
}

SPAT_MESSAGE_ID = 19


class FakeSpatNode(Node):
    def __init__(self):
        super().__init__("fake_spat")
        dyn = ParameterDescriptor(dynamic_typing=True)
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default, dyn)
        self.names: List[str] = [n for n in (self.get_parameter("signals.names").value or []) if n]
        for n in self.names:
            for fld, default in SIGNAL_FIELDS.items():
                self.declare_parameter(f"signals.{n}.{fld}", default, dyn)
        self._read_signals()  # fail early if the YAML file is incomplete

        self.event_of = {
            GREEN: MovementEvent.PROTECTED_MOVEMENT_ALLOWED,
            YELLOW: MovementEvent.PROTECTED_CLEARANCE,
            RED: MovementEvent.STOP_AND_REMAIN,
        }
        self.t0 = time.monotonic()
        self.last_state: Dict[str, str] = {}
        self.warned = False

        gp = lambda n: self.get_parameter(n).value
        spat_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_spat = self.create_publisher(SpatPacket, gp("topics.spat"), spat_qos)
        self.pub_tl = None
        if any(s["traffic_light_group_id"] > 0 for s in self._read_signals()):
            if TrafficLightGroupArray is None:
                self.get_logger().warn(
                    "traffic_light_group_id is set, but autoware_perception_msgs/"
                    "TrafficLightGroupArray is not available: publishing SPaT only")
            else:
                self.pub_tl = self.create_publisher(
                    TrafficLightGroupArray, gp("topics.traffic_signals"), 1)
        self.create_service(Trigger, "~/restart", self.on_restart)
        self.create_timer(1.0 / float(gp("rate_hz")), self.on_timer)
        self.get_logger().info(
            f"fake SPaT: {len(self.names)} signal(s) [{', '.join(self.names) or 'none configured'}]"
            + (", also publishing Autoware traffic signals" if self.pub_tl else ""))

    def _read_signals(self) -> List[Dict[str, float]]:
        """Read the signal table from the parameters (so runtime changes apply)."""
        out = []
        for n in self.names:
            sig: Dict[str, Any] = {"name": n}
            for fld in SIGNAL_FIELDS:
                v = self.get_parameter(f"signals.{n}.{fld}").value
                if v is None or isinstance(v, bool) or not isinstance(v, (int, float)):
                    raise ValueError(f"signals.{n}.{fld} must be a number")
                sig[fld] = float(v)
            if min(sig["green_s"], sig["yellow_s"], sig["red_s"]) <= 0.0:
                raise ValueError(f"signals.{n}: green_s, yellow_s and red_s must be > 0")
            out.append(sig)
        return out

    def on_restart(self, request, response):
        self.t0 = time.monotonic()
        response.success = True
        response.message = "all cycles restarted"
        self.get_logger().info("cycles restarted")
        return response

    def on_timer(self):
        try:
            signals = self._read_signals()
            self.warned = False
        except ValueError as e:
            if not self.warned:
                self.get_logger().error(f"not publishing: {e}")
                self.warned = True
            return
        elapsed = time.monotonic() - self.t0
        utc = time.time()
        groups = []
        for s in signals:
            state, rem = cycle_state(elapsed + s["offset_s"], s["green_s"], s["yellow_s"], s["red_s"])
            self.pub_spat.publish(self._spat_packet(s, state, rem, utc))
            if self.last_state.get(s["name"]) != state:
                self.last_state[s["name"]] = state
                self.get_logger().info(f"{s['name']}: {state} for {rem:.1f} s")
            if self.pub_tl is not None and s["traffic_light_group_id"] > 0:
                groups.append(self._traffic_light_group(int(s["traffic_light_group_id"]), state))
        if self.pub_tl is not None and groups:
            msg = TrafficLightGroupArray()
            msg.stamp = self.get_clock().now().to_msg()
            msg.traffic_light_groups = groups
            self.pub_tl.publish(msg)

    def _spat_packet(self, s: Dict[str, float], state: str, rem: float, utc: float) -> SpatPacket:
        ms_in_minute = int((utc % 60.0) * 1000.0)
        end = time_mark(utc, rem)

        timing = TimeChangeDetails()
        timing.has_min_end_time = True
        timing.min_end_time = end
        timing.has_max_end_time = True
        timing.max_end_time = end          # fixed-time: min == max

        event = MovementEvent()
        event.event_state = self.event_of[state]
        event.timing = timing

        movement = MovementState()
        movement.signal_group = int(s["signal_group"])
        movement.events.append(event)

        inter = IntersectionState()
        inter.intersection_id = int(s["intersection_id"])
        inter.revision = 0
        inter.moy = minute_of_year(utc)
        inter.time_stamp = ms_in_minute
        inter.states.append(movement)

        spat = Spat()
        spat.time_stamp = ms_in_minute
        spat.intersections.append(inter)

        out = SpatPacket()
        out.message_id = SPAT_MESSAGE_ID
        out.spat = spat
        return out

    @staticmethod
    def _traffic_light_group(group_id: int, state: str):
        el = TrafficLightElement()
        el.color = {GREEN: TrafficLightElement.GREEN, YELLOW: TrafficLightElement.AMBER,
                    RED: TrafficLightElement.RED}[state]
        el.shape = TrafficLightElement.CIRCLE
        el.status = TrafficLightElement.SOLID_ON
        el.confidence = 1.0
        g = TrafficLightGroup()
        g.traffic_light_group_id = group_id
        g.elements = [el]
        return g


def main():
    rclpy.init()
    node = FakeSpatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
