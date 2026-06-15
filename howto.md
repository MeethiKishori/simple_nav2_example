---
# Mode 1: SLAM — build a map by driving the robot manually

# Terminal 1 — Gazebo
source /opt/ros/jazzy/setup.bash
source ~/shubham/wicon/simple_nav2_example/install/setup.bash
ros2 launch nav2_mobile_robot gazebo.launch.py

# Terminal 2 — SLAM (wait for Gazebo to fully load first)
source /opt/ros/jazzy/setup.bash
source ~/shubham/wicon/simple_nav2_example/install/setup.bash
ros2 launch nav2_mobile_robot slam.launch.py

# Terminal 3 — RViz
source /opt/ros/jazzy/setup.bash
source ~/shubham/wicon/simple_nav2_example/install/setup.bash
ros2 run rviz2 rviz2 -d ~/shubham/wicon/simple_nav2_example/src/nav2_mobile_robot/rviz/rviz.rviz

# Terminal 4 — Teleop (drive the robot to build the map)
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel

# When done mapping, save the map:
ros2 run nav2_map_server map_saver_cli -f ~/shubham/wicon/simple_nav2_example/src/nav2_mobile_robot/map/map


---
# Mode 2: Autonomous Nav2 — robot drives itself using the pre-built map

# Terminal 1 — Gazebo
source /opt/ros/jazzy/setup.bash
source ~/shubham/wicon/simple_nav2_example/install/setup.bash
ros2 launch nav2_mobile_robot gazebo.launch.py

# Terminal 2 — AMCL + Map Server (localization with pre-built map)
source /opt/ros/jazzy/setup.bash
source ~/shubham/wicon/simple_nav2_example/install/setup.bash
ros2 launch nav2_mobile_robot amcl.launch.py

# Terminal 3 — Nav2 stack (planner + controller)
source /opt/ros/jazzy/setup.bash
source ~/shubham/wicon/simple_nav2_example/install/setup.bash
ros2 launch nav2_mobile_robot navigation.launch.py use_sim_time:=true

# Terminal 4 — RViz
source /opt/ros/jazzy/setup.bash
source ~/shubham/wicon/simple_nav2_example/install/setup.bash
ros2 run rviz2 rviz2 -d ~/shubham/wicon/simple_nav2_example/src/nav2_mobile_robot/rviz/rviz.rviz


# In RViz:
# 1. Change Fixed Frame to: map
# 2. Add Map display → Topic: /map → Durability: Transient Local
# 3. Add LaserScan display → Topic: /lidar
# 4. Click "2D Pose Estimate" → click where robot is on the map (sets AMCL start position)
# 5. Wait ~5 seconds for localization
# 6. Click "Nav2 Goal" → click destination on the map → robot plans and drives there
