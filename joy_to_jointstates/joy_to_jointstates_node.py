import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped
from math import pi, copysign
#import numpy as np #Python doesn't have sign() function for some fkin reason 

sign = lambda x: copysign(1, x) #Better - doesn't add dependency on numpy, but still WTF

from manipulator_servo_driver_interfaces.srv import ChangeMode

SPEED_MULTIPLIER_MIN = 0.0
SPEED_MULTIPLIER_MAX = 1.0

ROTATION_MULTIPLIER = 10 
LINEAR_MULTIPLIER = -0.1 

TEST_JOINT_POS = [-0.050633996262358316,
                  0.5370243852290245,
                  1.018811976548778,
                  -0.3007336557282536,
                  0.09206132318211835
                  ]

PICKUP_POS = [-0.306871,
              1.422347,
              0.955903,
              -0.198699,
              0.045647]

STORE_POS = [0.128885,
            -0.599932,
            -1.321079,
            -0.200233,
            0.045647]

STANDBY_POS = [-0.033756,
               -1.135422,
                1.775249,
                0.635990,
                0.054086]

GRIPPER_POS_MIN = 10
GRIPPER_POS_MAX = 100

class JoyToJointStates(Node):
    def __init__(self):
        super().__init__('joy_to_jointstates')
        self.joint_count = 5

        self.declare_parameter('max_speeds', [1.0]*self.joint_count)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('set_joints_velocity_topic', 'manipulator/set_joints_velocity')
        self.declare_parameter('gripper_position_topic', 'gripper_cmd_pos')
        self.declare_parameter('ik_vel_topic', 'kinematics/set_velocity')
        self.declare_parameter('joint_states_topic','joint_states')
        self.declare_parameter('set_joints_position_topic', 'manipulator/set_joints_position')

        self.max_speeds = self.get_parameter('max_speeds').get_parameter_value().double_array_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        set_joint_velocity_topic = self.get_parameter('set_joints_velocity_topic').get_parameter_value().string_value
        gripper_position_topic = self.get_parameter('gripper_position_topic').get_parameter_value().string_value
        ik_vel_topic = self.get_parameter('ik_vel_topic').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        set_joints_position_topic = self.get_parameter('set_joints_position_topic').get_parameter_value().string_value


        self.sub = self.create_subscription(Joy, joy_topic, self.cb, 1)
        self.pub = self.create_publisher(JointState, set_joint_velocity_topic, 1)
        self.gripper_pub = self.create_publisher(Int32, gripper_position_topic, 1)
        self.ik_vel_pub = self.create_publisher(TwistStamped, ik_vel_topic, 1)
        self.joint_states_sub = self.create_subscription(JointState, joint_states_topic, self.joint_states_cb, 1)
        self.set_joints_position_pub = self.create_publisher(JointState, set_joints_position_topic, 1)

        self.change_mode_client = self.create_client(ChangeMode, "manipulator/change_mode")


        self.act_joint_pos_rad = [0.0] * self.joint_count

        self.names = [f'joint_{i+1}' for i in range(5)]
        self.speed_multiplier = 0.2
        self.speed_multiplier_changed = False

        self.current_gripper_pos = 10

        # Manual control mode
        # 0 - Joint control
        # 1 - IK 
        # 2 - PreDefined - Go to predefined pose
        # 3 - Off
        self.mode = 0
        self.mode_changed = False 
        self.pos_sent = False

        # Joint mode
        # 0 - standby (not used here)
        # 1 - position
        # 2 - velocity
        self.joint_mode = 2

        # Make sure joint mode is set to default (velocity)
        req = ChangeMode.Request()
        req.mode = 2

        self.change_mode_client.call_async(req)

        
        syncmsg = Int32()
        syncmsg.data = self.current_gripper_pos
        self.gripper_pub.publish(syncmsg)

    def cb(self, msg: Joy):
        # Mode change
 
        if msg.buttons[0] == 0 and msg.buttons[2] == 0 and msg.buttons[3] == 0 and msg.buttons[1] == 0:
            self.mode_changed = False

        # PreDefined
        elif msg.buttons[0] == 1 and not self.mode_changed:
            # Switch to joints to position mode
            if (self.mode != 2):
                req = ChangeMode.Request()
                req.mode = 1 

                self.change_mode_client.call_async(req)
                #TODO: Add response resolution

            self.mode = 2
            self.mode_changed = True
        
        elif not self.mode_changed:
            if self.mode == 2:
                req = ChangeMode.Request()
                req.mode = 2 

                self.change_mode_client.call_async(req)
                #TODO: Add response resolution

            #Joint control
            if msg.buttons[2] == 1:
                self.mode = 0
                self.mode_changed = True

            #Hybrid
            elif msg.buttons[3] == 1:
                self.mode = 1
                self.mode_changed = True

            # Off
            elif msg.buttons[1] == 1:
                self.mode = 3

                self.mode_changed = True      


        if self.mode_changed == True:
            self.get_logger().info(f"Changed mode to {self.mode}")
            

        # Speed multiplier 
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


        # Joint control mode
        if self.mode == 0:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.names

            js.velocity = [0.0] * 5

            js.velocity[0] = msg.axes[2] * self.max_speeds[0] * self.speed_multiplier
            js.velocity[1] = msg.axes[3] * self.max_speeds[1] * self.speed_multiplier
            js.velocity[2] = msg.axes[1] * self.max_speeds[2] * self.speed_multiplier
            js.velocity[3] = msg.axes[6] * self.max_speeds[3] * self.speed_multiplier
            js.velocity[4] = -msg.axes[5] * self.max_speeds[4] * self.speed_multiplier
            
            self.get_logger().info(f'Publishing velocities: {js.velocity}')  
            self.pub.publish(js)

        # IK mode
        if self.mode == 1:

            twist_st = TwistStamped()

            # Header
            twist_st.header.stamp = self.get_clock().now().to_msg()
            twist_st.header.frame_id = "base-tool"

            # Linear velocities
            twist_st.twist.linear.x = msg.axes[3] * self.speed_multiplier * LINEAR_MULTIPLIER
            twist_st.twist.linear.y = msg.axes[2] * self.speed_multiplier * LINEAR_MULTIPLIER
            twist_st.twist.linear.z = msg.axes[1] * self.speed_multiplier * LINEAR_MULTIPLIER

            # Angular velocities
            twist_st.twist.angular.x = -msg.axes[5] * self.max_speeds[4] * self.speed_multiplier * ROTATION_MULTIPLIER
            twist_st.twist.angular.y = msg.axes[6] * self.max_speeds[3] * self.speed_multiplier * ROTATION_MULTIPLIER
            #twist_st.twist.angular.z = msg.axes[2] * self.max_speeds[0] * self.speed_multiplier * ROTATION_MULTIPLIER * 10

            # Logging
            self.get_logger().info(
                f'Publishing velocities: \n linear:{twist_st.twist.linear} \n angular={twist_st.twist.angular}'
            )

            # Publish
            self.ik_vel_pub.publish(twist_st)
        
        #Off
        if self.mode == 3:
           self.get_logger().info(
                f'Joystick control is disabled'
            ) 
        
        #PreDefined

        if self.mode == 2:
            if not self.pos_sent:
                if msg.axes[6] > 0.2:
                    self.go_to_pos(PICKUP_POS)

                elif msg.axes[6] < -0.2:
                    self.go_to_pos(STORE_POS)

                elif msg.axes[5] > 0.2:
                    self.go_to_pos(STANDBY_POS)

            elif abs(msg.axes[6]) < 0.1:
               self.pos_sent = False

        # Gripper control
        if self.current_gripper_pos < GRIPPER_POS_MAX:
            self.current_gripper_pos = self.current_gripper_pos + msg.buttons[6]

        if self.current_gripper_pos > GRIPPER_POS_MIN:
            self.current_gripper_pos = self.current_gripper_pos - msg.buttons[7]

        gripperpos_msg = Int32()
        gripperpos_msg.data = self.current_gripper_pos
        self.gripper_pub.publish(gripperpos_msg)


    def joint_states_cb(self, msg: JointState):
        self.act_joint_pos_rad = msg.position

    def go_to_pos(self, joint_pos):
        deltaPos = [0.0] * self.joint_count

        #Ignore differences less than 1° 
        for i in range(self.joint_count):
            delta = joint_pos[i] - self.act_joint_pos_rad[i]
            if abs(delta) > (1*pi/180):
                deltaPos[i] = delta
            
        if all([ delta == 0 for delta in deltaPos]):
            self.get_logger().info("Arm already in requested position")
            return
        self.get_logger().info(f"Calculated: {deltaPos}")

        velocity = [self.max_speeds[i] * self.speed_multiplier for i in range(self.joint_count)]

        joint_msg = JointState()
        joint_msg.position = deltaPos
        joint_msg.velocity = velocity
        self.set_joints_position_pub.publish(joint_msg)
        self.get_logger().info(f"Going to position")
        self.pos_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = JoyToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
