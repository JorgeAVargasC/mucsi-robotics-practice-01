# ur3e_kinematics

## Before each exercise

```powershell
# Windows - leave this terminal running the simulation
.\lab.bat iniciar
.\lab.bat sim 203
```

RViz: <http://localhost:6081> (password `laboratorio`)

Open each container terminal with `.\lab.bat terminal`, then run:

```bash
cd ~/ros_workspace
source install/setup.bash
```

Build once, before the first run:

```bash
colcon build --symlink-install --packages-select ur3e_kinematics_interfaces ur3e_kinematics
source install/setup.bash
```

## ex_01: read the joint states

```bash
# Container T1
ros2 run ur3e_kinematics ex_01_joint_states_subscriber
```

Expected: one log per second with the arm positions.

## ex_02: run the trajectories

```bash
# Container T1
ros2 run ur3e_kinematics ex_02_trajectory_executor
```

Expected: in RViz, the robot runs the joint, linear and circular trajectories, returning home between them.

## Record trajectory data

Start the recorder before running `ex_02_trajectory_executor`:

```bash
ros2 run ur3e_kinematics kinematics_recorder
```

In another container terminal, run the executor as shown above. Stop the
recorder with `Ctrl+C` after the executor finishes. It writes a CSV in
`~/ros_workspace` with a `trajectory` column (`joint`, `linear` or
`circular`). Only samples while an arm joint moves faster than 0.001 rad/s are
recorded. This threshold ignores small fluctuations, but may omit very slow
motion. Moves to home and idle periods between trajectories are not recorded.
The first acceleration after a stop is left empty because there is no previous
moving sample; velocity can also be empty when it must be estimated.

## ex_04: publish the custom message

```bash
# Container T1
ros2 run ur3e_kinematics ex_04_joint_kinematics_publisher

# Container T2
ros2 topic echo /joint_kinematics

# Container T3 - move the robot
ros2 run ur3e_kinematics ex_02_trajectory_executor
```

Expected: T2 shows positions, velocities and accelerations in one message, changing while the robot moves.

## Stop

`Ctrl+C` in each terminal, then in Windows: `.\lab.bat parar`
