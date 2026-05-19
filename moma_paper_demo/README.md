# moma_paper_demo

Holistic reactive whole-body control for the MiR + UR10e mobile manipulator,
implementing the algorithm from:

> **"A Holistic Approach to Reactive Mobile Manipulation"**
> Haviland, Sünderhauf, Corke — IEEE RA-L Vol. 7 No. 2, April 2022.
> DOI: 10.1109/LRA.2022.3146554

The controller solves a Quadratic Program (QP) at each timestep to simultaneously
command the MiR base and UR10e arm toward a desired end-effector pose in the map frame,
maximising arm manipulability and avoiding joint limits throughout the motion.

---

## Prerequisites

### 1. ROS 2 dependencies (rosdep)

```bash
rosdep install --from-paths src/moma_paper_demo --ignore-src -r -y
```

### 2. Python dependencies (pip — not in rosdep)

```bash
pip install -r src/moma_paper_demo/requirements.txt
```

This installs:
| Package | Version | Purpose |
|---|---|---|
| `roboticstoolbox-python` | ≥ 1.1.0 | UR10 kinematics, Jacobian, manipulability |
| `spatialmath-python` | ≥ 1.0.0 | SE3 / SO3 math (installed with RTB-P) |
| `qpsolvers` | ≥ 4.0.0 | QP solver interface |
| `osqp` | ≥ 0.6.0 | QP backend (open-source, real-time capable) |

---

## Build

```bash
colcon build --symlink-install --packages-select moma_paper_demo
source install/setup.bash
```

---

## Running

Start the three launch files **in order** (each in its own terminal):

**Simulation:**
```bash
# Terminal 1 — Gazebo, robot_state_publisher, ros2_control
ros2 launch moma_bringup moma_system.launch.py use_sim:=true

# Terminal 2 — Nav2 + MoveGroup + RViz (wait for Terminal 1 to be fully up)
ros2 launch moma_bringup moma_nav_moveit.launch.py use_sim:=true \
  map:=/home/phumint/moma_ws/src/moma_navigation/maps/my_room_map.yaml

# Terminal 3 — MoveIt Servo + whole-body controller (wait for MoveGroup to be ready)
ros2 launch moma_paper_demo demo.launch.py use_sim:=true
```

**Real hardware:**
```bash
# Terminal 1
ros2 launch moma_bringup moma_system.launch.py use_sim:=false

# Terminal 2
ros2 launch moma_bringup moma_nav_moveit.launch.py use_sim:=false \
  map:=/home/phumint/moma_ws/src/moma_navigation/maps/perron_hallway_rightSide.yaml

# Terminal 3
ros2 launch moma_paper_demo demo.launch.py use_sim:=false
```

`demo.launch.py` starts both MoveIt Servo (`servo_node_main`) and the QP controller node.
MoveIt Servo converts `JointJog` commands from the controller into `JointTrajectory`
messages for `ur_manipulator_controller`, so no separate servo launch step is needed.

### Changing the goal

Edit `config/demo_params.yaml` — `goal_pose` is the desired end-effector pose
in the `map` frame (position + quaternion).

---

## Running Tests

Unit tests for the controller algorithm require `roboticstoolbox-python` and
`qpsolvers` to be installed.

```bash
# Unit tests only (no ROS required)
python3 -m pytest src/moma_paper_demo/test/ -v

# Full ament test suite
colcon test --packages-select moma_paper_demo
colcon test-result --verbose
```

---

## Architecture

```
/joint_states ──────────────────────────────┐
                                            │
TF: map → base_footprint ──────────────┐   │
TF: map → ur_tool0 ────────────────────┤   │
TF: base_footprint → ur_base_link ─────┤   ▼
  (cached on first iteration)       ControllerNode
                                        │
                                  WholeBodyController
                               (whole_body_controller.py)
                                        │
                           ┌────────────┴────────────┐
                           ▼                         ▼
                    /cmd_vel                /servo_node/delta_joint_cmds
               (geometry_msgs/Twist)       (control_msgs/JointJog)
                    MiR base                   MoveIt Servo → UR10e
```

### Key files

| File | Description |
|---|---|
| `moma_paper_demo/whole_body_controller.py` | Pure Python QP controller — no ROS. Implement paper changes here. |
| `moma_paper_demo/controller_node.py` | Thin ROS 2 wrapper: TF + joint_states → controller → publishers |
| `config/demo_params.yaml` | All tunable parameters (goal, QP gains, tolerances) |
| `launch/demo.launch.py` | `use_sim:=true/false` is the only switch between sim and real |
| `ALGORITHM_NOTES.md` | Full mathematical derivation, QP matrix construction, parameter guide |
