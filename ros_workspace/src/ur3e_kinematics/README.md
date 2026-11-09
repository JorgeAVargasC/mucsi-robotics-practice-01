# ur3e_kinematics

Práctica 1: registrar la posición, velocidad y aceleración de las articulaciones
del UR3e mientras el robot ejecuta una trayectoria joint, una lineal y una circular.

## Nodos

| Nodo | Qué hace |
| --- | --- |
| `trajectory_executor` | Mueve el robot con MoveIt: home -> joint -> home -> lineal -> home -> circular -> home. Publica la trayectoria activa en `/active_trajectory`. |
| `kinematics_recorder` | Se suscribe a `/joint_states` y `/active_trajectory`. Guarda las muestras en un CSV y las publica en `/joint_kinematics` (`ur3e_kinematics_interfaces/msg/JointKinematics`). |

## Cómo probarlo

Se usan 4 terminales: T1 en Windows y T2, T3 y T4 dentro del contenedor
(cada una se abre con `.\lab.bat terminal`).

```powershell
# T1 (Windows) - arrancar la simulación y dejarla corriendo
.\lab.bat iniciar
.\lab.bat sim 203
```

RViz en el navegador: <http://localhost:6081/vnc.html?autoconnect=true&password=laboratorio&resize=scale>

```bash
# T2 (.\lab.bat terminal) - compilar una vez y arrancar el recorder
cd ~/ros_workspace
colcon build --symlink-install --packages-select ur3e_kinematics_interfaces ur3e_kinematics
source install/setup.bash
ros2 run ur3e_kinematics kinematics_recorder
```

```bash
# T3 (.\lab.bat terminal) - opcional, ver el mensaje personalizado
cd ~/ros_workspace
source install/setup.bash
ros2 topic echo /joint_kinematics
```

```bash
# T4 (.\lab.bat terminal) - ejecutar las trayectorias (después del recorder)
cd ~/ros_workspace
source install/setup.bash
ros2 run ur3e_kinematics trajectory_executor
```

Cuando T4 termine, pulsar `Ctrl+C` en T2 para cerrar el CSV.

## Resultado esperado

- RViz: el robot ejecuta las trayectorias joint, lineal y circular, y vuelve a
  home entre una y otra.
- T3: un mensaje `JointKinematics` por cada muestra guardada (posiciones,
  velocidades y aceleraciones), solo mientras el robot ejecuta una trayectoria.
- CSV: `~/ros_workspace/kinematics_<fecha>_<hora>.csv`, una fila por muestra,
  con una columna `trajectory` (`joint`, `linear` o `circular`) y el timestamp,
  la posición, la velocidad y la aceleración de cada articulación.

Solo se guardan las muestras en las que alguna articulación del brazo se mueve
a más de 0,001 rad/s. Las vueltas a home y los periodos quietos entre
trayectorias no se guardan. La primera aceleración después de una parada queda
vacía porque no hay una muestra anterior en movimiento; la velocidad también
puede quedar vacía cuando hay que estimarla.

## Tests

```bash
cd ~/ros_workspace
colcon test --packages-select ur3e_kinematics
colcon test-result --verbose
```

## Detener

`Ctrl+C` en cada terminal y luego, en Windows: `.\lab.bat parar`
