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
                'max_speeds': [0.2, 2.0, 3.0, 0.2, 0.2],   # scale factors for each axis
                'joy_topic': '/freya_1/chassis/manipulator/joy',
                'joint_states_topic': '/manipulator/set_joints_velocity',
                'gripper_position_topic': '/freya_1/chassis/gripper/cmd_pos'
            }]
        )
    ])
