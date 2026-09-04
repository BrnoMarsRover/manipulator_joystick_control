import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from math import pi, copysign
from std_srvs.srv import Trigger
#import numpy as np #Python doesn't have sign() function for some fkin reason 

sign = lambda x: copysign(1, x) #Better - doesn't add dependency on numpy, but still WTF

from ament_index_python.packages import get_package_share_directory
import os
import xacro
from urdf_parser_py.urdf import URDF


SPEED_MULTIPLIER_MIN = 0.0
SPEED_MULTIPLIER_MAX = 1.0

ROTATION_MULTIPLIER = 2.5
LINEAR_MULTIPLIER = 0.25

GRIPPER_POS_MIN = 0.0
GRIPPER_POS_MAX = 1.0

class JoyToJointStates(Node):
    def __init__(self):
        super().__init__('joy_to_jointstates')

        self.getUrdf()

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
        self.gripper_pub = self.create_publisher(Float32, gripper_position_topic, 1)
        self.ik_vel_pub = self.create_publisher(TwistStamped, ik_vel_topic, 1)
        self.set_joints_position_pub = self.create_publisher(JointState, set_joints_position_topic, 1)

        self.zero_wrist_service = self.create_client(Trigger, "/freya_1/manipulator/kinematics/zero_wrist")

        self.act_joint_pos_rad = [0.0] * self.joint_count

        self.speed_multiplier = 0.2
        self.speed_multiplier_changed = False

        self.current_gripper_pos = 0.0

        # Manual control mode
        # 0 - Joint control
        # 1 - IK - base coordinate system
        # 2 - IK - tool coordinate system
        # 3 - Off
        self.mode = 3
        self.mode_changed = False 
        self.pos_sent = False

        # Joint mode
        # 0 - standby (not used here)
        # 1 - position
        # 2 - velocity
        self.joint_mode = 2

        
        syncmsg = Float32()
        syncmsg.data = self.current_gripper_pos
        self.gripper_pub.publish(syncmsg)

    def cb(self, msg: Joy):
        # Mode change
 
        if msg.buttons[0] == 0 and msg.buttons[2] == 0 and msg.buttons[3] == 0 and msg.buttons[1] == 0:
            self.mode_changed = False
        
        elif not self.mode_changed:
            
            #Joint control
            if msg.buttons[1] == 1:
                self.mode = 0
                self.mode_changed = True

            #IK
            elif msg.buttons[0] == 1:
                self.mode = 1
                self.mode_changed = True

            elif msg.buttons[3] == 1:
                self.mode = 2
                self.mode_changed = True

            # Off
            elif msg.buttons[2] == 1:
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
            js.name = self.joint_names

            js.velocity = [0.0] * self.joint_count

            js.velocity[0] = msg.axes[2] * self.max_speeds[0] * self.speed_multiplier
            js.velocity[1] = msg.axes[3] * self.max_speeds[1] * self.speed_multiplier
            js.velocity[2] = msg.axes[1] * self.max_speeds[2] * self.speed_multiplier
            js.velocity[3] = msg.axes[5] * self.max_speeds[3] * self.speed_multiplier
            js.velocity[4] = -msg.axes[4] * self.max_speeds[4] * self.speed_multiplier

            _w = self.servo_from_wrist([js.velocity[-2], js.velocity[-1]])
            js.velocity[-2] = _w[0]
            js.velocity[-1] = _w[1]
            
            self.get_logger().info(f'Publishing velocities: {js.velocity}')  
            self.pub.publish(js)

        # IK mode
        if self.mode == 1:

            twist_st = TwistStamped()
            twist_st.header.frame_id = "base"

            # Header
            twist_st.header.stamp = self.get_clock().now().to_msg()

            # Linear velocities
            twist_st.twist.linear.x = msg.axes[3] * self.speed_multiplier * LINEAR_MULTIPLIER
            twist_st.twist.linear.y = msg.axes[2] * self.speed_multiplier * LINEAR_MULTIPLIER
            twist_st.twist.linear.z = msg.axes[1] * self.speed_multiplier * LINEAR_MULTIPLIER

            # Angular velocities
            twist_st.twist.angular.x = msg.axes[4] * self.max_speeds[4] * self.speed_multiplier * ROTATION_MULTIPLIER
            twist_st.twist.angular.y = msg.axes[5] * self.max_speeds[3] * self.speed_multiplier * ROTATION_MULTIPLIER
            #twist_st.twist.angular.z = msg.axes[2] * self.max_speeds[0] * self.speed_multiplier * ROTATION_MULTIPLIER * 10

            # Logging
            self.get_logger().info(
                    f'Publishing velocities in base frame: \n linear:{twist_st.twist.linear} \n angular={twist_st.twist.angular}'
                )
            
            # Publish
            self.ik_vel_pub.publish(twist_st)

        if self.mode == 2:

            twist_st = TwistStamped()
            twist_st.header.frame_id = "tool"

            # Header
            twist_st.header.stamp = self.get_clock().now().to_msg()

            # Linear velocities
            twist_st.twist.linear.x = msg.axes[3] * self.speed_multiplier * LINEAR_MULTIPLIER
            twist_st.twist.linear.y = msg.axes[2] * self.speed_multiplier * LINEAR_MULTIPLIER
            twist_st.twist.linear.z = msg.axes[1] * self.speed_multiplier * LINEAR_MULTIPLIER

            # Angular velocities
            twist_st.twist.angular.z = msg.axes[4] * self.max_speeds[4] * self.speed_multiplier * ROTATION_MULTIPLIER
            twist_st.twist.angular.y = msg.axes[5] * self.max_speeds[3] * self.speed_multiplier * ROTATION_MULTIPLIER
            #twist_st.twist.angular.z = msg.axes[2] * self.max_speeds[0] * self.speed_multiplier * ROTATION_MULTIPLIER * 10

            # Logging
            self.get_logger().info(
                    f'Publishing velocities in tool frame: \n linear:{twist_st.twist.linear} \n angular={twist_st.twist.angular}'
                )
            
            # Publish
            self.ik_vel_pub.publish(twist_st)
            

            self.get_logger().info(
                f'Publishing velocities in tool frame: \n linear:{twist_st.twist.linear} \n angular={twist_st.twist.angular}'
                )

        #Off
        if self.mode == 3:
           self.get_logger().info(
                f'Joystick control is disabled'
            ) 

        # Gripper control
        if self.current_gripper_pos < GRIPPER_POS_MAX:
            self.current_gripper_pos = self.current_gripper_pos + msg.buttons[6]

        if self.current_gripper_pos > GRIPPER_POS_MIN:
            self.current_gripper_pos = self.current_gripper_pos - msg.buttons[7]

        gripperpos_msg = Float32()
        gripperpos_msg.data = self.current_gripper_pos
        self.gripper_pub.publish(gripperpos_msg)

        # Zero wrist service call
        if msg.buttons[9] == 1:
            self.zero_wrist_service.call_async(Trigger.Request())
            self.get_logger().info("Zero wrist service called")

    def getUrdf(self):
        self.get_logger().info("Probíhá generování kinematického řetězce")

        package_share_directory = get_package_share_directory('roboarm_description')

        # Generated fresh from the xacro rather than reading the committed
        # urdf/roboarm.urdf, so the xacro is the only source of truth and the
        # model can never silently go stale against it.
        #
        # TracIK takes a FILE path, not a string, so the result is written to a
        # temp file and that path is used for both the parser and the solver.
        # Kept for the process lifetime (delete=False): TracIK reads it during
        # construction below, and it is small.
        xacro_path = os.path.join(package_share_directory, 'urdf', 'roboarm.urdf.xacro')
        try:
            urdf_content = xacro.process_file(xacro_path).toxml()
        except Exception as exc:
            self.get_logger().fatal(
                f"Nelze zpracovat xacro {xacro_path}: {exc}. ")
            raise

        self.get_logger().info(f"URDF vygenerovano z {xacro_path}")
        

        # Vytvoření struktury robota z URDF - xml stringu
        robot = URDF.from_xml_string(urdf_content)

        #Save joint names from urdf
        self.joint_count = 0
        self.joint_names = []
        for joint in robot.joints:
            if joint.type != "fixed":
                self.joint_names.append(joint.name)
                self.joint_count += 1

        self.get_logger().info(f"Klouby: {self.joint_names}")

    @staticmethod
    def servo_from_wrist(wristAngle: list[float]):
        servoAngle0 = (wristAngle[0] + wristAngle[1])
        servoAngle1 = (wristAngle[0] - wristAngle[1])
        
        #return wristAngle # <-- ignoruje kinematiku zápěstí a řídí serva samostatně jako předtím
        return [servoAngle0, servoAngle1]


    @staticmethod
    def wrist_from_servo(servoAngle :list[float]):
        wristAngle0 = (servoAngle[0] + servoAngle[1]) / 2
        wristAngle1 = (servoAngle[0] - servoAngle[1]) / 2

        #return servoAngle # <-- ignoruje kinematiku zápěstí a řídí serva samostatně jako předtím
        return [wristAngle0, wristAngle1]
    
def main(args=None):
    rclpy.init(args=args)
    node = JoyToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
