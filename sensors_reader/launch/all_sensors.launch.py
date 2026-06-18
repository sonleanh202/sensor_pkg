from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def make_sensor_node(executable: str, yaml_name: str):
    return Node(
        package='sensors_reader',
        executable=executable,
        name=executable,
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('sensors_reader'),
                'config',
                'sensors',
                yaml_name,
            ])
        ],
    )


def generate_launch_description():
    nodes = [
        make_sensor_node('co_sensor_node', 'co_sensor.yaml'),
        make_sensor_node('co2_sensor_node', 'co2_sensor.yaml'),
        make_sensor_node('ch4_sensor_node', 'ch4_sensor.yaml'),
        make_sensor_node('o2_sensor_node', 'o2_sensor.yaml'),
        make_sensor_node('smoke_fire_alarm_node', 'smoke_fire_alarm_sensor.yaml'),
        make_sensor_node('humidity_sensor_node', 'humidity_sensor.yaml'),

        # SEN0568 H2S reads through Arduino Nano USB serial, not RS485.
        make_sensor_node('h2s_arduino_node', 'h2s_arduino_sensor.yaml'),

        make_sensor_node('sensor_monitor_node', 'sensor_monitor.yaml'),
    ]

    return LaunchDescription(nodes)
