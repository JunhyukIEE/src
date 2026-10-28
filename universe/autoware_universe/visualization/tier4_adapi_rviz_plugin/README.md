# tier4_adapi_rviz_plugin

This package contains tools for testing AD API. For general AD API usage, we recommend using [tier4_state_rviz_plugin](../tier4_state_rviz_plugin/README.md).

## RoutePanel

To use the panel, set the topic name from 2D Goal Pose Tool to `/rviz/routing/pose`.
By default, when a tool publish a pose, the panel immediately sets a route with that as the goal.
Enable or disable of allow_goal_modification option can be set with the check box.

Push the mode button in the waypoint to enter waypoint mode. In this mode, the pose is added to waypoints.
Press the apply button to set the route using the saved waypoints (the last one is a goal).
Reset the saved waypoints with the reset button.

## K-City Goal Setter

`tier4_adapi_rviz_plugins::GoalSetterPanel` sets the competition checkpoints with one
click **before** enabling Auto. It is included in the launch RViz configuration and
docks on the left. Initialize localization near START/END, stop in manual mode,
clear any old route, click **Goal Setter: K-City 1 lap**, wait for **Ready** in its tooltip, then
press Auto. No further user interaction is required for the route updates.

- Checkpoints and headings are in `src/kcity_course.hpp`. Local coordinates are
  translated to UTM zone 52N using `(302595, 4124145)`, then to the current
  `MGRS 52SCG` map frame by subtracting `(300000, 4100000)`. Heights are unchanged;
  headings are grid yaw in degrees. Other map projections disable the button.
- The mission planner rejects routes containing repeated lanelets. The initial
  route therefore uses checkpoints 1–13 as waypoints and checkpoint 14 as a
  temporary goal. After passing checkpoint 2, the panel automatically requests
  the remaining checkpoints, START/END as a waypoint, and a separate stop goal
  60 m beyond END along the road centerline of `K_city_26_crosswork_1.osm`.
  `course[0]` and END scoring are unchanged. The extra distance removes the need
  to stop at END; it does not override speed limits, curves, signals or obstacles,
  and does not guarantee a 60 km/h crossing speed.
- Checkpoints are counted in order within an 8 m radius, or across an 8 m-wide
  gate on either side of the checkpoint heading. Opposite-direction passes and
  localization jumps greater than 30 m between samples are ignored. Waypoints do
  not request intermediate stops.
- Updates use `/api/routing/change_route_points` and retain the planner's reroute
  safety checks. A rejected update is retried after 2 s without clearing the old
  route. An unanswered request is not duplicated. The panel does not engage,
  disengage, or bypass vehicle control or safety checks.
- **Keep RViz and this panel open during the lap.** Closing the panel stops its
  automatic updates. If no extension succeeds, the existing route still ends at
  checkpoint 14; the panel reports an incomplete mission. Clearing the route or
  changing its goal externally cancels automatic updates, not vehicle control.
- `/rviz/goal_setter/checkpoints` displays the numbered checkpoints, START/END,
  and a separate orange `STOP GOAL` marker (not a scored checkpoint).

### Speed-limit warning

The panel passively monitors `/vehicle/status/velocity_status` and
`/localization/kinematic_state`, independently of Goal Setter and Auto engagement.
The supplied UTM 52N endpoints `(302657.074, 4124401.495)` and
`(302665.214, 4123779.252)` are converted to MGRS 52SCG map coordinates.
The straight corridor between their perpendicular boundary lines, within 20 m
on either side, is displayed as unrestricted. The start is inclusive and the end
is exclusive; other positions use a 60 km/h display limit. This is a course-specific
geofence, not a general lanelet speed-limit lookup. Position refers to `base_link`.

Outside that corridor, any reported absolute longitudinal speed greater than
60 km/h produces a red warning without tolerance or speed averaging. A brief
exceedance and its peak stay visible for 2 s after the last exceeding sample.
Entering the unrestricted corridor clears the warning. Missing/nonfinite inputs,
an incompatible map or over 500 ms without either input show a grey waiting state.
No velocity limits, control commands, checkpoints or route goals are changed.
Localization error and sensor reporting latency affect the boundary judgement;
this display is not a guarantee against speeding or the official scoring system.

### Checkpoint result lights

The panel also shows CP 1–14 and END inside colored circles, with a success count out of 15. Green means
the vehicle reference point passed within **3 m in map x/y**; grey means pending;
red means the checkpoint was missed. Fresh consecutive odometry samples are
linearly interpolated to catch passes between samples. Gaps over 500 ms and pose
jumps over 30 m are not interpolated. This display-only scoring does not change
the existing 8 m route-progress logic, routing services, TF, or localization.

A near miss is marked red when the vehicle crosses 3 m beyond the checkpoint's
heading plane within 20 m of that checkpoint. Reaching a later checkpoint also
marks any skipped earlier checkpoints red. END is evaluated only after CP 14
has been resolved, so passing START on departure cannot complete the lap.

Results remain visible after END. Once the vehicle stops (absolute speed below
0.1 m/s) and departs again (above 0.5 m/s), results automatically reset to grey.
The **통과 기록 초기화** button resets only the displayed results, not the route or
control mode; it is enabled only when stopped outside active Auto control.
Setting a new course with Goal Setter also starts a fresh result record.

Coordinate/order tests can be run with:

```sh
colcon test --packages-select tier4_adapi_rviz_plugin --ctest-args -R test_kcity_course
```

The supplied checkpoints were checked against `K_city_26_crosswork_1.osm` for road
membership and routing connectivity. This does not replace an end-to-end simulator
test of automatic rerouting and checkpoint passage.

## Material Design Icons

This project uses [Material Design Icons](https://developers.google.com/fonts/docs/material_symbols) by Google. These icons are used under the terms of the Apache License, Version 2.0.

Material Design Icons are a collection of symbols provided by Google that are used to enhance the user interface of applications, websites, and other digital products.

### License

The Material Design Icons are licensed under the Apache License, Version 2.0. You may obtain a copy of the License at:

<http://www.apache.org/licenses/LICENSE-2.0>

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

### Acknowledgments

We would like to express our gratitude to Google for making these icons available to the community, helping developers and designers enhance the visual appeal and user experience of their projects.
