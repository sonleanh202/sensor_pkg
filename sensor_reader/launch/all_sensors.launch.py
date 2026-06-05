from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def config_file(yaml_name: str):
    return PathJoinSubstitution([
        FindPackageShare('sensor_reader'),
        'config',
        'sensors',
        yaml_name,
    ])


def make_sensor_node(executable: str, yaml_name: str):
    return Node(
        package='sensor_reader',
        executable=executable,
        name=executable,
        output='screen',
        parameters=[config_file(yaml_name)],
    )


def generate_launch_description():
    return LaunchDescription([
        make_sensor_node('co_sensor_node', 'co_sensor.yaml'),
        make_sensor_node('co2_sensor_node', 'co2_sensor.yaml'),
        make_sensor_node('ch4_sensor_node', 'ch4_sensor.yaml'),
        make_sensor_node('o2_sensor_node', 'o2_sensor.yaml'),
        make_sensor_node('smoke_fire_alarm_node', 'smoke_fire_alarm_sensor.yaml'),
        make_sensor_node('h2s_sensor_node', 'h2s_sensor.yaml'),
        make_sensor_node('humidity_sensor_node', 'humidity_sensor.yaml'),

        Node(
            package='sensor_reader',
            executable='sensor_monitor_node',
            name='sensors_reader',
            output='screen',
            parameters=[config_file('sensor_monitor.yaml')],
        ),
    ])