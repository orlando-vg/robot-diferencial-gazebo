# Robot diferencial en Gazebo con LiDAR y cámara

Universidad EIA – Ingeniería Mecatrónica – Robótica (Entrega 1)

Robot móvil de tracción diferencial modelado en URDF/Xacro y simulado en **ROS 2 Jazzy** con **Gazebo Harmonic**. Tiene un LiDAR 2D y una cámara, recorre una pista propia mediante teleoperación por teclado, y un nodo propio calcula la odometría a partir de la cinemática de las llantas y la compara con la del plugin DiffDrive.

## Paquetes

| Paquete | Tipo | Contenido |
|---|---|---|
| `mi_robot_diferencial` | ament_cmake | Modelo del robot (`urdf/`), pista (`worlds/`), bridge (`config/`), launch (`launch/`) y configuración de RViz2 (`rviz/`) |
| `mi_robot_cinematica` | ament_python | Nodo `odom_ruedas` (cinemática de las llantas) y script `scripts/graficar_error.py` |

## Dependencias

Ubuntu 24.04, ROS 2 Jazzy y Gazebo Harmonic. Paquetes adicionales:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt-image-view ros-jazzy-tf2-tools \
  python3-matplotlib
```

## Compilar

```bash
cd ~/ros2_ws_dif
colcon build
source install/setup.bash
```

## Ejecutar

**Terminal 1** – levanta Gazebo con la pista, el robot, robot_state_publisher, el bridge, RViz2 y el nodo de cinemática:

```bash
ros2 launch mi_robot_diferencial mi_robot.launch.py
```

**Terminal 2** – teleoperación por teclado:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
```

Teclas: `i` avanzar, `j`/`l` girar, `u`/`o` avanzar girando, `k` frenar, `,` retroceder.

## Odometría por ruedas

El nodo `odom_ruedas` lee `w = [wD, wI]` de `/joint_states`, calcula `u = [v, ω] = M·w` y `q̇ = J(θ)·u`, integra la pose y la compara con `/odom`. Publica `/odom_ruedas` y `/error_odom`, e imprime el error cada segundo.

Los datos se guardan en `~/error_odom.csv`. Para generar las gráficas (después de detener el launch):

```bash
python3 ~/ros2_ws_dif/src/mi_robot_cinematica/scripts/graficar_error.py
```

## Topics principales

| Topic | Tipo | Descripción |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | Velocidad del robot (ROS → Gazebo) |
| `/odom` | nav_msgs/Odometry | Odometría del plugin DiffDrive |
| `/joint_states` | sensor_msgs/JointState | Posición y velocidad de las ruedas |
| `/scan` | sensor_msgs/LaserScan | LiDAR |
| `/image_raw`, `/camera_info` | sensor_msgs/Image, CameraInfo | Cámara |
| `/odom_ruedas` | nav_msgs/Odometry | Odometría del nodo propio |
| `/error_odom` | std_msgs/Float64MultiArray | Error `[ex, ey, eθ, e_pos]` entre ambas odometrías |

## Documentación

Explicación del modelo, la cinemática, los parámetros y las decisiones de diseño: [enlace al Notion]