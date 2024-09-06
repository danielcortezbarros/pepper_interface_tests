import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([

        # Declare launch arguments
        DeclareLaunchArgument(
            'nao_ip', default_value='172.29.111.230',
            description='IP address of the NAO or Pepper robot.'
        ),
        DeclareLaunchArgument(
            'qi_listen_url', default_value='tcp://0.0.0.0:0',
            description='The Qi listen URL for NAOqi.'
        ),
        DeclareLaunchArgument(
            'namespace', default_value='naoqi_driver',
            description='Namespace for the driver.'
        ),
        DeclareLaunchArgument(
            'launch_audio_nodes', default_value='false',
            description='Whether to launch audio nodes.'
        ),

        # Launch the naoqi_driver_node
        launch_ros.actions.Node(
            package='naoqi_driver',
            executable='naoqi_driver_node',
            name=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'qi_url': LaunchConfiguration('qi_listen_url'),
                'nao_ip': LaunchConfiguration('nao_ip')
            }]
        ),

        # Conditionally launch naoqiAudioPublisher node
        GroupAction([
            launch_ros.actions.Node(
                package='naoqi_driver',
                executable='naoqiAudioPublisher.py',
                name='naoqiAudioPublisher',
                output='screen'
            )
        ], condition=IfCondition(LaunchConfiguration('launch_audio_nodes'))),

        # Conditionally launch naoqiAudio node (Python 2 using shell script)
        GroupAction([
            launch.actions.ExecuteProcess(
                cmd=['run_naoqiAudio.sh', '--ip', LaunchConfiguration('nao_ip')],
                name='naoqiAudio',
                output='screen',
            )
        ], condition=IfCondition(LaunchConfiguration('launch_audio_nodes')))
    ])
