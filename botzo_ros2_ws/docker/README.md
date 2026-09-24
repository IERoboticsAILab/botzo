# Botzo ROS 2 Jazzy in Docker (+ Foxglove instead of RViz)

Works on macOS (Apple Silicon, Docker Desktop) and Linux. Everything ROS runs
inside one container; you look at it with [Foxglove](https://app.foxglove.dev)
through `foxglove_bridge` on `ws://localhost:8765`.

## First time

```bash
# from the repo root
docker compose -f botzo_ros2_ws/docker/compose.yaml up -d --build
docker exec botzo bash -lc 'cd /ws && colcon build --symlink-install'
```

The repo is bind-mounted at `/ws/src/botzo`, and `--symlink-install` means edits
to Python nodes on your host are live after restarting the node (rebuild only
when you add files, entry points or messages).

## Walk: kinematic only (no physics)

```bash
docker exec -it botzo bash
ros2 launch botzo_bringup walk.launch.py                # new trot planner
ros2 launch botzo_bringup walk.launch.py gait:=legacy   # old planner, to compare
```

## Walk: physics (Gazebo Harmonic, headless)

```bash
ros2 launch botzo_gazebo sim.launch.py                  # same control stack, real contacts
ros2 run botzo_gazebo gait_eval                         # scripted test drive + score table
```

Foxglove layout for the sim: `botzo_bringup/foxglove/botzo_sim_layout.json`
(grid fixed to `odom`, so you see the robot walk across the floor).

`gait_eval` drives forward / turn / strafe / backward / stop and prints, per
segment, the speed actually reached, drift, body tilt, body bob and whether the
robot fell. To compare settings: `ros2 param set /gait_planner <name> <value>`
after the sim is up, then run `gait_eval` again.

Open Foxglove -> *Open connection* -> *Foxglove WebSocket* -> `ws://localhost:8765`,
then *Layouts -> Import from file* -> `botzo_ros2_ws/botzo_bringup/foxglove/botzo_walk_layout.json`.
The Teleop panel publishes `/cmd_vel` (arrows: forward/back, turn left/right;
releasing an arrow sends a stop).

If you build your own layout instead: in the 3D panel enable the
`/robot_description` topic and set *Scene -> Mesh up-axis* to **Z-up**. The STL
meshes are Z-up, and Foxglove's default (Y-up) draws the robot lying on its side.
Foxglove stops drawing while its browser tab is in the background; that is not
the stack dying.

From a terminal instead:

```bash
docker exec -it botzo bash -lc 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'
# or a one-off command
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

Tune the gait live (all fields of `gait_core.GaitParams`, defaults in
`botzo_bringup/config/gait.yaml`):

```bash
ros2 param set /gait_planner period 0.4
ros2 param set /gait_planner step_height 3.0
ros2 param set /gait_planner home_bl "[2.0, 4.0, 16.0]"   # per-leg home, IK frame [x, y, z] cm
```

## Tests

```bash
docker exec botzo bash -lc 'cd /ws/src/botzo/botzo_ros2_ws && python3 -m pytest -q botzo_gaitplan/test botzo_gazebo/test'
```
