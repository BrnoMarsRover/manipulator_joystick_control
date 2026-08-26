from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            remappings=[('joy', '/freya_1/chassis/manipulator/joy')]
        ),
        Node(
            package='joy_to_jointstates',
            executable='joy_to_jointstates',
            name='joy_to_jointstates',
            parameters=[{
                'max_speeds': [2.0, 2.0, 2.0, 2.0, 2.0],   # scale factors for each axis
                'joy_topic': '/freya_1/chassis/manipulator/joy',
                'joint_states_topic': '/freya_1/manipulator_driver/cubemars_node/command',
                'gripper_position_topic': '/st3215_gripper/command'
            }]
        )
    ])
