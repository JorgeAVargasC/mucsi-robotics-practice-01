# Commands

- **Windows** = PowerShell or cmd in the repo folder
- **Container** = `.\lab.bat terminal`

## Start the simulation

```powershell
# Windows - T1
.\lab.bat iniciar
.\lab.bat sim 203
```

RViz: (password `laboratorio`)

```
http://localhost:6081
```

In every new container terminal
`.\lab.bat terminal`

```bash
cd ~/ros_workspace
source install/setup.bash
```

## After a change

| Change                                 | What to do                  |
| -------------------------------------- | --------------------------- |
| Edit an old `.py` file                 | Nothing: run the node again |
| New `.py` · `setup.py` · `package.xml` | Build again (code below)    |

```bash
# Container · build again
cd ~/ros_workspace
colcon build --symlink-install --packages-select ur3e_kinematics
source install/setup.bash
```

## `setup.py` and `package.xml`

Path: `ros_workspace/src/ur3e_kinematics/`

| File          | When to change it                                                        | What to add                   |
| ------------- | ------------------------------------------------------------------------ | ----------------------------- |
| `setup.py`    | I make a **new node** (a `.py` with `main()` that I run with `ros2 run`) | One line in `console_scripts` |
| `package.xml` | I `import` a **new ROS package**                                         | One `<depend>` line           |

**`setup.py`**: each line is `'command = package.file:function',`

```python
    entry_points={
        'console_scripts': [
            '<node_a> = ur3e_kinematics.<script_a>:main',
            '<node_b> = ur3e_kinematics.<script_b>:main',
            # <- new node here:  '<node> = ur3e_kinematics.<script>:main',
        ],
    },
```

Then: `ros2 run ur3e_kinematics <node>`

**`package.xml`**: one line for each package you import (use a tab at the start)

```xml
	<depend>rclpy</depend>
	<depend>sensor_msgs</depend>
	<depend>geometry_msgs</depend>
	<depend>moveit_msgs</depend>
	<depend>shape_msgs</depend>
	<depend>tf2_ros</depend>
	<!-- ← new package here:  <depend>std_msgs</depend> -->
```

| If you write this `import`                                  | Add this                                      |
| ----------------------------------------------------------- | --------------------------------------------- |
| `from std_msgs.msg import String`                           | `<depend>std_msgs</depend>`                   |
| `from ur3e_kinematics_interfaces.msg import ...`            | `<depend>ur3e_kinematics_interfaces</depend>` |
| Python modules (`math`, `csv`, `copy`…) or your own package | Nothing                                       |

If you change one of these two files -> **build again**.

## Test

```bash
ros2 run ur3e_kinematics <node>

# Container - is data coming?
ros2 topic hz /joint_states
```

## Before you commit

```bash
# Container
colcon test --packages-select ur3e_kinematics
colcon test-result --verbose
```

## Stop

`Ctrl+C` in each terminal -> Windows: `.\lab.bat parar`
