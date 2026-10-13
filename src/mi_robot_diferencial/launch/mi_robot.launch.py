"""
Levanta todo con un solo comando:
Gazebo con la pista + robot + robot_state_publisher + bridge + RViz2

    ros2 launch mi_robot_diferencial mi_robot.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ===== Rutas a los archivos del paquete (ya instalados con colcon build) =====
    pkg = get_package_share_directory('mi_robot_diferencial')
    xacro_path = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'pista.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'mi_robot.rviz')

    # Convierte el xacro a URDF al momento de lanzar
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    # ===== 1) Gazebo con la pista (-r = arranca corriendo, sin pausa) =====
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v4 {world_path}'}.items(),
    )

    # ===== 2) robot_state_publisher: publica /robot_description y los TF del robot =====
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # ===== 3) Spawn: pone el robot en el punto de salida (carril sur, mirando a +x) =====
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mi_robot',
                   '-x', '-2.6', '-y', '-2.4', '-z', '0.1', '-Y', '0.0'],
        output='screen',
    )

    # ===== 4) Bridge ROS 2 <-> Gazebo (topics definidos en config/bridge.yaml) =====
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    # ===== 5) RViz2 con la configuración guardada en la Parte 7 =====
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
    ])