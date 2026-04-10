import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import math

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.123
LX = 0.2045 
LY = 0.2225 
target_speed = {'x': 1.0, 'y': 1.0, 'z': 1.0}

class main:
    def init(self, webots_node, properties):
        print("===== init() called =====", flush=True)
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.fl_motor = self.__robot.getDevice('front_left_wheel_joint')
        self.fr_motor = self.__robot.getDevice('front_right_wheel_joint')
        self.bl_motor = self.__robot.getDevice('back_left_wheel_joint')
        self.br_motor = self.__robot.getDevice('back_right_wheel_joint')
        
        self.fl_motor.setPosition(float('inf'))
        self.fl_motor.setVelocity(0.0)
        self.fr_motor.setPosition(float('inf'))
        self.fr_motor.setVelocity(0.0)
        self.bl_motor.setPosition(float('inf'))
        self.bl_motor.setVelocity(0.0)
        self.br_motor.setPosition(float('inf'))
        self.br_motor.setVelocity(0.0)

        if not rclpy.ok():
            rclpy.init(args=None)

        self.namespace = properties.get('namespace', '')

        self.__node = rclpy.create_node(
            'robot_driver',
            namespace=self.namespace,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )

        self.odom_publisher = self.__node.create_publisher(Odometry, 'odom', 10)
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__target_twist = Twist()
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        self.clock_publisher = self.__node.create_publisher(Clock, '/clock', 10)

        self.gps = self.__robot.getDevice('gps')
        if self.gps:
            self.gps.enable(self.__timestep)

        self.imu = self.__robot.getDevice('imu')
        if self.imu:
            self.imu.enable(self.__timestep)

        print("===== init() done =====", flush=True)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Webots 시간 먼저 가져오기
        wb_time = self.__robot.getTime()

        # clock publish
        clock_msg = Clock()
        clock_msg.clock.sec = int(wb_time)
        clock_msg.clock.nanosec = int((wb_time - int(wb_time)) * 1e9)
        self.clock_publisher.publish(clock_msg)

        vx = self.__target_twist.linear.x
        vy = self.__target_twist.linear.y
        wz = self.__target_twist.angular.z

        fl, fr, bl, br = mecanumControl(vx, vy, wz)

        self.bl_motor.setVelocity(fl)
        self.fl_motor.setVelocity(fr)
        self.br_motor.setVelocity(bl)
        self.fr_motor.setVelocity(br)

        if self.gps and self.imu:
            gps_vals = self.gps.getValues()
            
            if gps_vals and not math.isnan(gps_vals[0]):
                curr_time = Time()
                curr_time.sec = int(wb_time)
                curr_time.nanosec = int((wb_time - int(wb_time)) * 1e9)

                t = TransformStamped()
                t.header.stamp = curr_time
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'

                t.transform.translation.x = float(gps_vals[0])
                t.transform.translation.y = float(gps_vals[1])
                t.transform.translation.z = float(gps_vals[2])

                rpy = self.imu.getRollPitchYaw()
                if rpy and not math.isnan(rpy[2]):
                    yaw = rpy[2]
                    t.transform.rotation.z = math.sin(yaw / 2.0)
                    t.transform.rotation.w = math.cos(yaw / 2.0)

                    odom_msg = Odometry()
                    odom_msg.header.stamp = curr_time
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link'

                    odom_msg.pose.pose.position.x = t.transform.translation.x
                    odom_msg.pose.pose.position.y = t.transform.translation.y
                    odom_msg.pose.pose.position.z = t.transform.translation.z
                    
                    odom_msg.pose.pose.orientation.z = t.transform.rotation.z
                    odom_msg.pose.pose.orientation.w = t.transform.rotation.w

                    self.odom_publisher.publish(odom_msg)
                    self.__tf_broadcaster.sendTransform(t)

def mecanumControl(vx, vy, wz):
    fl = 1 / WHEEL_RADIUS * (vx - vy - ((LY + LX) * wz))
    fr = 1 / WHEEL_RADIUS * (vx + vy - ((LY + LX) * wz))
    bl = 1 / WHEEL_RADIUS * (vx + vy + ((LY + LX) * wz))
    br = 1 / WHEEL_RADIUS * (vx - vy + ((LY + LX) * wz))
    return fl, fr, bl, br