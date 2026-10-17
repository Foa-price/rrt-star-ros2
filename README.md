# RRT* motion planning in ROS 2 Humble

Coursework project (MGTU STANKIN, ROS 2 seminars): an RRT* planner implemented
from scratch in Python and tested in Gazebo Classic on two robots.

| |  mobile robot | KUKA YouBot arm |

| Configuration space | 2D (x, y) | 5D (joint angles) |
| Obstacle | circle, r = 0.35 m | sphere, r = 0.10 m |
| Nodes explored | 708 | 2999 |
| Planning time | 0.19 s | 10.4 s |
| Result | 3.12 m path, goal reached | 22.8 mm final end-effector error |

Going from 2 to 5 dimensions made planning about 50x slower.

## Contents
- `rrt_star_nav/`: ROS 2 package with both planners
  (`rrt_star_planner` for the mobile robot, `rrt_star_arm_planner` for the arm).
  The arm planner uses forward kinematics built from the YouBot URDF and
  checks collisions on interpolated points along each link.
- `youbot_fixes/`: a bug I diagnosed in the course's YouBot Gazebo model (below). It contains the relay node, the fixed launch file and `apply_sdf_fix.sh`.
- `patches/`: diff of my changes to the mobile robot model (`basic_mobile_robot`).

## The bug: a TF tree split in two
Rosbags recorded with the stock YouBot model had no usable `/tf` from `world`
to the arm. Debugging with `tf2_echo` and `view_frames` showed two
disconnected trees, caused by two problems in the SDF:
1. `gazebo_ros_p3d` was configured with placeholder values
   (`body_name=pen`, `frame_name=ground_plane`).
2. `gazebo_ros_joint_state_publisher` publishes joint names with a Gazebo scope
   prefix (`youbot::arm_joint_1`), so `robot_state_publisher` could not match
   them with the URDF joint names (`arm_joint_1`).

Fix: `joint_state_debridge.py` republishes `/youbot/joint_states` without the
prefix on `/youbot/joint_states_clean`, and `robot_state_publisher` is
remapped to that topic. Verified with `tf2_echo world arm_link_5`, live and
during rosbag playback.

## Run
Needs the course workspace (https://github.com/MrBoriska/ros2_seminars_ws)
for the robot models. Copy `rrt_star_nav/` into its `src/`, copy the files of
`youbot_fixes/` into `youbot_description/` (the node into `scripts/`, the launch
file into `launch/`, and register the node in `CMakeLists.txt`), run
`youbot_fixes/apply_sdf_fix.sh` on the `model.sdf`, then:
```bash
colcon build --symlink-install
ros2 launch youbot_description youbot_gazebo.launch.py
ros2 run rrt_star_nav rrt_star_arm_planner
```
Rosbags of both runs are attached to the Releases page.

## What I learned
With this work, I was able to observe that RRT* is asymptotically optimal: it converges toward the globally optimal solution instead of settling for the first path found, as the classic RRT does.

The rewiring step proved crucial for this, as it constantly re-evaluates the neighbors of a new node in order to integrate it via the least-cost path, thereby optimizing the overall trajectory over time.

I was also able to measure the impact of the curse of dimensionality: going from 2 to 5 dimensions made planning about 50x slower, because random sampling covers the space less and less efficiently as the state space grows.

Finally, I found the TF bug by inspecting the frame tree with `view_frames` and `tf2_echo`: the tree was split in two disconnected parts, because the joint names published by Gazebo carried a `youbot::` prefix that did not match the URDF joint names.
