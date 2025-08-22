import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32

SPEED_MULTIPLIER_MIN = 0.0
SPEED_MULTIPLIER_MAX = 1.0

GRIPPER_POS_MIN = 10
GRIPPER_POS_MAX = 100

class JoyToJointStates(Node):
    def __init__(self):
        super().__init__('joy_to_jointstates')
        self.declare_parameter('max_speeds', [1.0]*5)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('gripper_position_topic', 'gripper_cmd_pos')

        self.max_speeds = self.get_parameter('max_speeds').get_parameter_value().double_array_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        gripper_position_topic = self.get_parameter('gripper_position_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(Joy, joy_topic, self.cb, 1)
        self.pub = self.create_publisher(JointState, joint_states_topic, 1)
        self.gripper_pub = self.create_publisher(Int32, gripper_position_topic, 1)
        self.names = [f'joint_{i+1}' for i in range(5)]

        self.speed_multiplier = 0.2
        self.speed_multiplier_changed = False

        self.current_gripper_pos = 10

        syncmsg = Int32()
        syncmsg.data = self.current_gripper_pos
        self.gripper_pub.publish(syncmsg)

    def cb(self, msg: Joy):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.names

        js.velocity = [0.0] * 5

        if msg.buttons[10] == 1 and self.speed_multiplier < SPEED_MULTIPLIER_MAX and not self.speed_multiplier_changed:
            self.speed_multiplier = self.speed_multiplier + 0.2
            self.speed_multiplier_changed = True
            self.get_logger().info(f'Changed the speed multiplier to {self.speed_multiplier}')

        if msg.buttons[11] == 1 and self.speed_multiplier > SPEED_MULTIPLIER_MIN and not self.speed_multiplier_changed:
            self.speed_multiplier = self.speed_multiplier - 0.2
            self.speed_multiplier_changed = True
            self.get_logger().info(f'Changed the speed multiplier to {self.speed_multiplier}')

        if msg.buttons[10] == 0 and msg.buttons[11] == 0:
            self.speed_multiplier_changed = False

        js.velocity[0] = msg.axes[2] * self.max_speeds[0] * self.speed_multiplier
        js.velocity[1] = -msg.axes[3] * self.max_speeds[1] * self.speed_multiplier
        js.velocity[2] = msg.axes[1] * self.max_speeds[2] * self.speed_multiplier
        js.velocity[3] = msg.axes[4] * self.max_speeds[3] * self.speed_multiplier
        
        # Rotation down
        if msg.axes[5] == -1.0:
            js.velocity[3] = self.max_speeds[4] * self.speed_multiplier
            js.velocity[4] = self.max_speeds[4] * self.speed_multiplier
        
        # Rotation up
        if msg.axes[5] == 1.0:
            js.velocity[3] = -self.max_speeds[4] * self.speed_multiplier
            js.velocity[4] = -self.max_speeds[4] * self.speed_multiplier

        # Rotation left
        if msg.axes[4] == 1.0:
            js.velocity[3] = self.max_speeds[3] * self.speed_multiplier
            js.velocity[4] = -self.max_speeds[3] * self.speed_multiplier

        if msg.axes[4] == -1.0:
            js.velocity[3] = -self.max_speeds[3] * self.speed_multiplier
            js.velocity[4] = self.max_speeds[3] * self.speed_multiplier

        self.get_logger().info(f'Publishing velocities: {js.velocity}')  
        self.pub.publish(js)

        if self.current_gripper_pos < GRIPPER_POS_MAX:
            self.current_gripper_pos = self.current_gripper_pos + msg.buttons[6]

        if self.current_gripper_pos > GRIPPER_POS_MIN:
            self.current_gripper_pos = self.current_gripper_pos - msg.buttons[7]

        gripperpos_msg = Int32()
        gripperpos_msg.data = self.current_gripper_pos
        self.gripper_pub.publish(gripperpos_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
