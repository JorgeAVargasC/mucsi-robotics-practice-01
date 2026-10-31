# Commands

- **Container** = `.\lab.bat terminal`

## Build and check the message

```bash
# Container
cd ~/ros_workspace
colcon build --packages-select ur3e_kinematics_interfaces
source install/setup.bash
ros2 interface show ur3e_kinematics_interfaces/msg/JointKinematics
```
