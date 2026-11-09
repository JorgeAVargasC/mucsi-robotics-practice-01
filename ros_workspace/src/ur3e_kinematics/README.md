# ur3e_kinematics

Practice 1: record the position, velocity and acceleration of the UR3e joints
while the robot runs a joint, a linear and a circular trajectory.

## Nodes

| Node | What it does |
| --- | --- |
| `trajectory_executor` | Moves the robot with MoveIt: home -> joint -> home -> linear -> home -> circular -> home. Publishes the active trajectory on `/active_trajectory`. |
| `kinematics_recorder` | Subscribes to `/joint_states` and `/active_trajectory`. Saves the samples to a CSV and publishes them on `/joint_kinematics` (`ur3e_kinematics_interfaces/msg/JointKinematics`). |

## How to test

Use 4 terminals. T1 is Windows; T2, T3 and T4 are container terminals
(`.\lab.bat terminal`).

```powershell
# T1 (Windows) - start the simulation and leave it running
.\lab.bat iniciar
.\lab.bat sim 203
```

RViz in the browser: <http://localhost:6081/vnc.html?autoconnect=true&password=laboratorio&resize=scale>

```bash
# T2 - build once, then start the recorder
cd ~/ros_workspace
colcon build --symlink-install --packages-select ur3e_kinematics_interfaces ur3e_kinematics
source install/setup.bash
ros2 run ur3e_kinematics kinematics_recorder
```

```bash
# T3 (optional) - see the custom message
cd ~/ros_workspace
source install/setup.bash
ros2 topic echo /joint_kinematics
```

```bash
# T4 - run the trajectories (start it after the recorder)
cd ~/ros_workspace
source install/setup.bash
ros2 run ur3e_kinematics trajectory_executor
```

When T4 finishes, press `Ctrl+C` in T2 to close the CSV file.

## Expected result

- RViz: the robot runs the joint, linear and circular trajectories and goes
  back home between them.
- T3: one `JointKinematics` message per saved sample (positions, velocities and
  accelerations), only while the robot runs a trajectory.
- CSV: `~/ros_workspace/kinematics_<date>_<time>.csv`, one row per sample, with a
  `trajectory` column (`joint`, `linear` or `circular`) and the timestamp,
  position, velocity and acceleration of each joint.

Only samples while an arm joint moves faster than 0.001 rad/s are recorded.
Moves to home and idle periods between trajectories are not recorded. The first
acceleration after a stop is empty because there is no previous moving sample;
velocity can also be empty when it must be estimated.

## Tests

```bash
cd ~/ros_workspace
colcon test --packages-select ur3e_kinematics
colcon test-result --verbose
```

## Stop

`Ctrl+C` in each terminal, then in Windows: `.\lab.bat parar`
