This project implements a differential drive mobile robot in simulation using ROS 2 and Gazebo. The robot can perform SLAM and autonomous navigation using the Nav2 stack.

> Forked from [MeethiKishori/simple_nav2_example](https://github.com/MeethiKishori/simple_nav2_example). Bug fixes applied to get Nav2 autonomous navigation working — see [Code changes from original](#code-changes-from-original-what-was-fixed) section.

> forged from abubakar-mughal97/nav2_mobile_robot

# Features

- Differential drive robot simulation
- LiDAR-based SLAM using SLAM Toolbox
- Autonomous navigation with Nav2
- Odometry and TF broadcasting for RViz visualization

# Tools & Technologies

- ROS 2 Jazzy
- Gazebo Harmonic
- SLAM Toolbox for mapping
- Nav2 stack for navigation
- RViz for visualization

---

# Mode 1: SLAM — build a map by driving manually

```bash
# Terminal 1 — Gazebo
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch nav2_mobile_robot gazebo.launch.py

# Terminal 2 — SLAM
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch nav2_mobile_robot slam.launch.py

# Terminal 3 — RViz
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run rviz2 rviz2 -d src/nav2_mobile_robot/rviz/rviz.rviz

# Terminal 4 — Teleop (drive around to build the map)
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

When done, save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f src/nav2_mobile_robot/map/map
```
Then rebuild: `colcon build --packages-select nav2_mobile_robot`

---

# Mode 2: Autonomous Nav2 — robot drives itself

Uses the pre-built map in `map/map.pgm`. No SLAM needed.

```bash
# Terminal 1 — Gazebo
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch nav2_mobile_robot gazebo.launch.py

# Terminal 2 — AMCL + Map Server (localization)
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch nav2_mobile_robot amcl.launch.py

# Terminal 3 — Nav2 stack (planner + controller)
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch nav2_mobile_robot navigation.launch.py use_sim_time:=true

# Terminal 4 — RViz
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run rviz2 rviz2 -d src/nav2_mobile_robot/rviz/rviz.rviz
```

### Set initial pose (required every time Gazebo starts) -- not urgent

The robot spawns at (0, 0) in Gazebo. Tell AMCL where it is:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{'header': {'frame_id': 'map'}, 'pose': {'pose': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'w': 1.0}}, 'covariance': [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}"
```
Or in RViz: click **2D Pose Estimate** → click where the robot is on the map.

### Send a navigation goal

From command line:
```bash
source /opt/ros/jazzy/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{'pose': {'header': {'frame_id': 'map'}, 'pose': {'position': {'x': 2.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'w': 1.0}}}}"
```
Or in RViz: click **2D Goal Pose** → click destination on the map.

### RViz setup (first time)
- Change Fixed Frame to `map`
- Add **Map** display → Topic: `/map` → Durability: `Transient Local`
- Add **LaserScan** display → Topic: `/lidar`
- Add **Path** display → Topic: `/plan` (shows the planned route)

---

# Code changes from original (what was fixed)

### 1. `config/nav.yaml` — observation source name mismatch
`local_costmap` had `observation_sources: scan` but the source block was named `lidar:` instead of `scan:`.
The costmap was silently ignoring all laser scan data, so local obstacle avoidance didn't work.
```yaml
# before
observation_sources: scan
lidar:          # ← wrong name, didn't match "scan"
  topic: /lidar

# after
observation_sources: scan
scan:           # ← matches the source name
  topic: /lidar
```

### 2. `launch/navigation.launch.py` — unconfigured nodes blocked lifecycle startup
`collision_monitor` and `docking_server` were listed in `lifecycle_nodes` but had no plugin configuration in `nav.yaml`.
The lifecycle_manager configures every node in the list on startup in order. Both nodes crashed during configure, which caused lifecycle_manager to abort — leaving `bt_navigator` and `planner_server` never activated, so navigation never worked.

**`collision_monitor`** — fixed by adding proper config to `nav.yaml` and adding it back to `lifecycle_nodes`. It reads the laser scan and cuts velocity if the robot is about to hit something (last-resort safety layer).

**`docking_server`** — permanently removed. It is for autonomously driving the robot onto a charging dock. This project has no dock in the simulation, so it serves no purpose here.

```python
# original (broken — both nodes had no config, lifecycle startup aborted)
lifecycle_nodes = [
    "controller_server", "smoother_server", "planner_server",
    "behavior_server", "velocity_smoother", "collision_monitor",
    "bt_navigator", "waypoint_follower", "docking_server",
]

# current (fixed)
lifecycle_nodes = [
    "controller_server", "smoother_server", "planner_server",
    "behavior_server", "velocity_smoother", "collision_monitor",  # ← re-added with config
    "bt_navigator", "waypoint_follower",                          # ← docking_server removed
]
```

### 3. `config/nav.yaml` — collision_monitor config added
`collision_monitor` was in the original `lifecycle_nodes` but had no config in `nav.yaml`, causing it to crash. Added a full config block so it can start properly. It bridges `cmd_vel_smoothed` → `cmd_vel`, completing the velocity pipeline without needing a manual relay.

```
controller → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel → Gazebo
                                                                        ↑
                                                                  watches /lidar
                                                                  stops robot if obstacle approaching
```
