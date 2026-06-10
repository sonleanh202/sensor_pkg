from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def make_sensor_node(executable: str, yaml_name: str):
    return Node(
        package='sensor_pkg',
        executable=executable,
        name=executable,
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('sensor_pkg'),
                'config',
                'sensors',
                yaml_name,
            ])
        ],
    )


def generate_launch_description():
    # H2S/SEN0467 is not a Modbus sensor. It uses DFRobot UART frames over the
    # same RS485 wiring, so let it run first and switch itself to passive mode
    # before the Modbus sensors start polling the shared /dev/ttyUSB0 bus.
    h2s_node = make_sensor_node('h2s_sensor_node', 'h2s_sensor.yaml')

    modbus_nodes = [
        make_sensor_node('co_sensor_node', 'co_sensor.yaml'),
        make_sensor_node('co2_sensor_node', 'co2_sensor.yaml'),
        make_sensor_node('ch4_sensor_node', 'ch4_sensor.yaml'),
        make_sensor_node('o2_sensor_node', 'o2_sensor.yaml'),
        make_sensor_node('smoke_fire_alarm_node', 'smoke_fire_alarm_sensor.yaml'),
        make_sensor_node('humidity_sensor_node', 'humidity_sensor.yaml'),
    ]

    monitor_node = make_sensor_node('sensor_monitor_node', 'sensor_monitor.yaml')

    return LaunchDescription([
        h2s_node,
        TimerAction(period=6.0, actions=modbus_nodes),
        TimerAction(period=8.0, actions=[monitor_node]),
    ])
