from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 150.0,
                        'coalesce_interval_ms': 0}],
            remappings=[('joy', '/manipulator/joy')]
        ),
        Node(
            package='joy_to_jointstates',
            executable='joy_to_jointstates',
            name='joy_to_jointstates',
            parameters=[{
                'max_speeds': [0.5, 0.5, 0.5, 0.5, 0.5],   # scale factors for each axis
                'joy_topic': '/manipulator/joy',
                'ik_vel_topic': '/freya_1/manipulator/kinematics/set_velocity',
                'gripper_position_topic': '/st3215_gripper/command',
                #'joint_states_topic': 'joint_states'
                'set_joints_velocity_topic': '/freya_1/manipulator_driver/cubemars_node/command'
            }]
        )
    ])
