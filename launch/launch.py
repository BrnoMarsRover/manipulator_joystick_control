from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            remappings=[('joy', '/manipulator/joy')]
        ),
        Node(
            package='joy_to_jointstates',
            executable='joy_to_jointstates',
            name='joy_to_jointstates',
            parameters=[{
                'max_speeds': [0.5, 0.5, 0.5, 0.5, 0.5],   # scale factors for each axis
                'joy_topic': '/manipulator/joy',
                'set_joints_velocity_topic': '/manipulator/set_joints_velocity',
                'gripper_position_topic': '/gripper/cmd_pos',
                #'joint_states_topic': 'joint_states'
            }]
        )
    ])
