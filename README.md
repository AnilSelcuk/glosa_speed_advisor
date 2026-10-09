# glosa_speed_advisor

GLOSA (Green Light Optimal Speed Advisory) for [Autoware](https://autoware.org), for fixed-time traffic signals.

The node reads the signal phase and timing (SPaT) of the traffic light ahead and, when it
helps, lowers the speed of the vehicle so that it reaches the stop line when the light is
green instead of stopping at red. If stopping cannot be avoided, it slows down early so the
wait at the red light is as short as possible.

It never drives faster than the map speed limit, and it never decides to stop: stopping at
red, obstacles and all other safety decisions stay in Autoware.

> **Status:** early version. The rule set is covered by unit tests. Validation on a real
> vehicle is still pending, so treat it as research code.

## How it works

```
SPaT (V2I bridge) ──┐
                    ├─► glosa_speed_advisor ─► max_velocity_candidates ─► external_velocity_limit_selector
ego pose and speed ─┘                                                            │ (lowest limit wins)
                                                                                 ▼
                                             Autoware planning ─────────► velocity_smoother ─► control
```

GLOSA does not touch the planning modules. It uses Autoware's external velocity limit: it
publishes a speed ceiling with the sender name `glosa`, and removes it again after the stop
line. Autoware applies the lowest
of all limits, so GLOSA can only make the vehicle slower, and only while it is approaching
a configured traffic light.

The decision is recomputed every time a SPaT message arrives (10 Hz). A watchdog timer
removes the limit if SPaT or the ego pose stops arriving.

### Rule set

`d` distance from the front bumper to the stop line, `v_map` lanelet speed limit, `T` time
until the signal changes, `G/Y/R` green, yellow and red durations, `t_map` time to reach
the stop line at `v_map`.

| Signal | Condition | Action |
|---|---|---|
| Green | `t_map + 2 s < T` | no limit, the green is reachable |
| Green | otherwise | aim for the next green: `T_green = T + Y + R` |
| Red | | `T_green = T` |
| Yellow | `d > 50 m` | `T_green = T + R` |
| Yellow | `d <= 50 m` | no limit, Autoware decides |

With `T_green` known:

- `t_map >= T_green + 1 s`: no limit, the map speed already arrives on green.
- otherwise: `limit = d / (T_green + 1 s)`, kept between `v_min` and `v_map` and rounded
  down to a 5 km/h step.

`v_min = max(10 km/h, 0.5 * v_map)`: it follows the speed limit of the lane, so a 35 km/h
lane gives 17.5 km/h and a 20 km/h lane 10 km/h.

Always: below 30 m the last decision is frozen until the stop line; stale SPaT, a stale ego
pose, an unknown signal state or a passed stop line remove the limit. All numbers above are
parameters.

## Requirements

- ROS 2 Humble and an Autoware workspace that provides `autoware_internal_planning_msgs`
- `v2i_spat_msgs` (the SPaT message package of the V2I bridge) in the same workspace
- A lanelet2 map whose points carry `local_x` / `local_y` tags, if stop lines are taken from the map

## Install

```bash
cd <your_ws>/src
git clone https://github.com/<user>/glosa_speed_advisor.git
cd <your_ws>
colcon build --symlink-install --packages-select glosa_speed_advisor
source install/setup.bash
```

With `--symlink-install`, edits to the YAML and launch files take effect without rebuilding.

## Quick start

**1. Find the traffic light in the map**

```bash
ros2 run glosa_speed_advisor list_traffic_lights /path/to/lanelet2_map.osm
```

```
traffic_light   lanelet     stop_x     stop_y  heading   limit
       173422   5006658    -428.35     170.44   -24.1°   35 km/h
       173422   5007394    -407.71     167.75   155.5°   35 km/h
```

Each row is one lane of a traffic light: the point where the lane meets the stop line, the
driving direction there and the lanelet speed limit. Add `--near X Y` to sort by distance
to a map point.

**2. Describe the approach in `config/glosa.param.yaml`**

```yaml
approaches:
  names: ["my_light"]

  my_light:
    intersection_id: 17342            # SPaT IntersectionID
    signal_group: 2                   # SPaT signalGroup of the ego lane
    green_s: 40.0                     # fixed-time plan of that signal group
    yellow_s: 4.0
    red_s: 46.0
    traffic_light_group_id: 173422    # ID from step 1
    speed_limit_kmh: 0                # 0 = use the lanelet speed limit
```

`intersection_id` and `signal_group` come from the SPaT message, `traffic_light_group_id`
from the map. This entry is what links the two.

**3. Start the node**

```bash
ros2 launch glosa_speed_advisor glosa_speed_advisor.launch.py \
    map_file:=/path/to/lanelet2_map.osm
```

At startup it prints the stop line it read for every lane. Then watch the decisions:

```bash
ros2 topic echo /glosa_speed_advisor/status
```

```
[limit: 20 km/h] my_light: d=250m v=30km/h RED T=40.0s -> red: slow down to arrive on green
```

## Try it in the planning simulator

The simulator has no SPaT, so the package includes a fake fixed-time SPaT publisher.

1. Start the Autoware planning simulator with your map. Place the vehicle about 300 m
   before a traffic light and set a goal behind it.
2. Describe the signal in `config/fake_spat.param.yaml`, with the same
   `intersection_id`, `signal_group` and durations as in `config/glosa.param.yaml`
   (see [Fake SPaT publisher](#fake-spat-publisher)).
3. Terminal A, fake SPaT:

   ```bash
   ros2 launch glosa_speed_advisor fake_spat.launch.py
   ```

4. Terminal B, GLOSA:

   ```bash
   ros2 launch glosa_speed_advisor glosa_speed_advisor.launch.py \
       map_file:=/path/to/lanelet2_map.osm
   ```

5. Engage the vehicle. When GLOSA sets a limit, the speed limit shown in RViz drops and
   the vehicle slows down; after the stop line the limit is removed again.

## Fake SPaT publisher

`fake_spat_node` plays a fixed-time green → yellow → red cycle and publishes it in the same
format as the real V2I bridge (`v2i_spat_msgs/SpatPacket`, 10 Hz, `minEndTime == maxEndTime`).
It exists so GLOSA can be tested without an RSU, for example in the planning simulator.

### 1. Describe the signal

Edit `config/fake_spat.param.yaml`:

```yaml
signals:
  names: ["my_light"]

  my_light:
    intersection_id: 17342          # must match glosa.param.yaml (0 - 65535)
    signal_group: 2                 # must match glosa.param.yaml (0 - 255)
    green_s: 40.0                   # must match glosa.param.yaml
    yellow_s: 4.0
    red_s: 46.0
    offset_s: 0.0                   # where in the cycle the signal starts
    traffic_light_group_id: 173422  # optional, 0 = publish SPaT only
```

| Field | Meaning |
|---|---|
| `intersection_id`, `signal_group` | the SPaT identity of the signal. GLOSA matches incoming SPaT to an approach by these two numbers, so they must be the same in both YAML files. In the simulator any numbers work as long as they match. |
| `green_s`, `yellow_s`, `red_s` | phase durations. Use the same values as in `glosa.param.yaml`. |
| `offset_s` | position in the cycle when the node starts. The cycle starts with green, so with 40 / 4 / 46 s: `0` = green just started, `40` = yellow starts, `44` = red starts, `60` = red with 30 s left. |
| `traffic_light_group_id` | ID of the traffic light in the lanelet2 map. When set, the same state is also sent to Autoware on `/perception/traffic_light_recognition/traffic_signals`, so the simulated vehicle stops at red and the light changes colour in RViz. Only this field is a map ID; `intersection_id` and `signal_group` are not. |

To simulate several signals, add more names to `names` and one block per name.

### 2. Start it

```bash
ros2 launch glosa_speed_advisor fake_spat.launch.py
```

The terminal prints a line at every phase change:

```
[fake_spat]: my_light: GREEN for 40.0 s
[fake_spat]: my_light: YELLOW for 4.0 s
[fake_spat]: my_light: RED for 46.0 s
```

Launch arguments:

| Argument | Default |
|---|---|
| `param_file` | `config/fake_spat.param.yaml` |
| `spat_topic` | `/v2i/spat/raw` |
| `traffic_signals_topic` | `/perception/traffic_light_recognition/traffic_signals` |

### 3. Check it

```bash
ros2 topic hz /v2i/spat/raw                     # about 10 Hz
ros2 topic echo /glosa_speed_advisor/status     # GLOSA shows RED / GREEN and T=...
```

If GLOSA reports `no SPaT`, the `intersection_id` or `signal_group` differ between the two
YAML files.

### 4. Control the cycle while it runs

Restart every cycle from its `offset_s`, for example at the moment the vehicle is engaged,
so that each test run sees the same signal timing:

```bash
ros2 service call /fake_spat/restart std_srvs/srv/Trigger
```

Change a value without restarting the node:

```bash
ros2 param set /fake_spat signals.my_light.red_s 30.0
ros2 param set /fake_spat signals.my_light.offset_s 60.0   # takes effect at once
```

### Notes

- Simulator only. On the vehicle it would publish on the same topic as the real SPaT
  bridge; never run both together.
- The Autoware traffic signal output uses `autoware_perception_msgs/TrafficLightGroupArray`.
  If that message type is not available in your Autoware version, the node prints a
  warning and publishes SPaT only.

## Configuration

Topic names and the map path are launch arguments. Everything else is in
`config/glosa.param.yaml`, where every parameter is documented.

### Launch arguments

| Argument | Default | Type |
|---|---|---|
| `param_file` | `config/glosa.param.yaml` | path |
| `map_file` | (empty) | path to `lanelet2_map.osm` |
| `spat_topic` | `/v2i/spat/raw` | `v2i_spat_msgs/SpatPacket` (input) |
| `odometry_topic` | `/localization/kinematic_state` | `nav_msgs/Odometry` (input) |
| `velocity_limit_topic` | `/planning/scenario_planning/max_velocity_candidates` | `autoware_internal_planning_msgs/VelocityLimit` (output) |
| `clear_velocity_limit_topic` | `/planning/scenario_planning/clear_velocity_limit` | `autoware_internal_planning_msgs/VelocityLimitClearCommand` (output) |
| `status_topic` | `/glosa_speed_advisor/status` | `std_msgs/String` (output) |

List them with `ros2 launch glosa_speed_advisor glosa_speed_advisor.launch.py --show-args`.

### Parameters you are most likely to tune

| Parameter | Default | Meaning |
|---|---|---|
| `rules.v_min_ratio` | 0.5 | lowest speed GLOSA may ask for, as a fraction of the lanelet speed limit |
| `rules.v_min_kmh` | 10 | `v_min` never goes below this |
| `rules.margin_green_start_s` | 1.0 | arrive at least this long after green starts |
| `rules.margin_green_end_s` | 2.0 | green counts as reachable only with this much time to spare |
| `rules.step_kmh` | 5.0 | the limit is rounded down to a multiple of this |
| `rules.freeze_distance_m` | 30 | below this distance the decision is frozen |
| `activation.distance_m` | 300 | GLOSA acts only within this distance of the stop line |
| `activation.require_fixed_time` | true | act only when SPaT `minEndTime == maxEndTime` |
| `vehicle.base_link_to_front_m` | 4.05 | rear axle to front bumper (wheel base + front overhang) |
| `output.use_constraints` | false | also send acceleration and jerk limits with the speed limit |

Most parameters can be changed while the node runs:

```bash
ros2 param set /glosa_speed_advisor rules.v_min_ratio 0.6
```

Those marked `[restart]` in the YAML file are read only at startup.

### Stop line by hand

Without a usable map, leave `traffic_light_group_id: 0` and give `stop_line_x`,
`stop_line_y`, `heading_deg` and `speed_limit_kmh` yourself. This helper prints the first
three from two clicks with the RViz "Publish Point" tool:

```bash
ros2 run glosa_speed_advisor stop_line_helper
```

## Reading the status topic

| Status text | Meaning |
|---|---|
| `no ego pose` | no odometry received yet |
| `no approach` | the vehicle is not within the activation distance of a configured stop line, or its heading does not match |
| `no SPaT` | no SPaT for this `intersection_id` / `signal_group` |
| `SPaT timeout` | SPaT stopped arriving, the limit was removed |
| `min != max (not fixed-time), skipped` | the signal is not fixed-time |
| `green reachable at map speed` | nothing to do |
| `slow down to arrive on green` | a limit is active |
| `cannot avoid red, minimize waiting` | the limit is `v_min`; the vehicle will still stop, but for a shorter time |
| `frozen` | inside the freeze distance, decision kept |

## Package contents

| File | Purpose |
|---|---|
| `glosa_speed_advisor/glosa_node.py` | ROS 2 node |
| `glosa_speed_advisor/glosa_core.py` | rule set, no ROS dependency |
| `glosa_speed_advisor/lanelet_osm.py` | reads traffic lights and stop lines from the lanelet2 map |
| `glosa_speed_advisor/fake_spat_node.py` | fake fixed-time SPaT publisher for the simulator |
| `glosa_speed_advisor/stop_line_helper.py` | stop line from two RViz clicks |
| `config/glosa.param.yaml` | settings of the GLOSA node |
| `config/fake_spat.param.yaml` | signal plan of the fake SPaT publisher |
| `launch/` | launch files of both nodes |
| `test/test_glosa_core.py` | unit tests |

## Limitations

- **Fixed-time signals only.** With adaptive signals the remaining time changes while the
  vehicle approaches; the node skips them by default.
- **Straight approaches.** The stop line comes from the map, but the distance to it is
  measured along a straight line, not along the route. If the road curves before the
  light, lower `activation.distance_m`.
- **Free-flow assumption.** The arrival time assumes the vehicle can drive at the map
  speed. A slower vehicle ahead makes the real arrival later than planned.
- **Slowing down only.** There is no speed-up advice and no coasting; deceleration uses
  Autoware's normal limits.
- **Signal plan in the YAML file.** The green, yellow and red durations are not read from
  SPaT. They are needed to aim for the next green while the light is green or yellow.

## Tests

```bash
cd <your_ws>/src/glosa_speed_advisor
python3 -m pytest -q
```

The tests need no ROS installation.

## License

Apache-2.0
