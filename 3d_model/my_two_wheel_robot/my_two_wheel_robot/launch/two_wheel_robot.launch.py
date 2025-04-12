import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('my_two_wheel_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_two_wheel_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    controller_params_file = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')

    # 1) Генерируем Substitution для xacro -> urdf
    robot_description_substitution = Command(['xacro ', xacro_file])
    # 2) Явно указываем, что это строка
    robot_description = ParameterValue(robot_description_substitution, value_type=str)
    # 3) Готовим словарь
    robot_description_param = {'robot_description': robot_description}

    # --- Узел robot_state_publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]  # <-- передаём dict
    )

    # --- Узел ROS2 Control (controller_manager) ---
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description_param,  # словарь (robot_description)
            controller_params_file    # путь к YAML
        ]
    )

    # Спавним контроллеры
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',  '--param-file', controller_params_file],
        output='screen',
    )

    # (Опционально) узел joint_state_publisher_gui
    # Для дифф. робота обычно не нужен, но если хотите видеть слайдеры, оставьте
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Собираем всё в LaunchDescription
    return LaunchDescription([
        rsp_node,
        ros2_control_node,
        joint_state_spawner,
        diff_drive_spawner,
        joint_state_publisher_node,
        rviz_node,
    ])
