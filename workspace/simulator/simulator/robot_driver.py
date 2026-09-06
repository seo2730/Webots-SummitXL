import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.123
LX = 0.2045 
LY = 0.2225 

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

        # 🌟 바퀴 관절 상태(joint_states) 발행 준비.
        # 이게 없으면 robot_state_publisher 가 바퀴 링크의 TF 를 못 만들고,
        # RViz 의 RobotModel 이 링크 TF 부재로 통째로 빨간 에러가 된다.
        # URDF 의 관절 이름과 Webots 디바이스 이름이 같아서 그대로 쓴다.
        #
        # 센서 이름을 추측하지 않고 모터에서 직접 얻는다. PositionSensor 가 없는
        # 모델이면 None 이 돌아오므로 그 바퀴만 조용히 빠진다.
        self.wheel_sensors = {}
        for joint_name, motor in (
            ('front_left_wheel_joint', self.fl_motor),
            ('front_right_wheel_joint', self.fr_motor),
            ('back_left_wheel_joint', self.bl_motor),
            ('back_right_wheel_joint', self.br_motor),
        ):
            sensor = motor.getPositionSensor()
            if sensor is not None:
                sensor.enable(self.__timestep)
                self.wheel_sensors[joint_name] = sensor

        if not rclpy.ok():
            rclpy.init(args=None)

        # 🌟 URDF에서 넘겨준 namespace 받기 (예: 'ugv1', 'ugv2')
        self.namespace = properties.get('namespace', '')
        
        # 🌟 동적 프레임 이름 생성 
        self.odom_frame = f"{self.namespace}/odom" if self.namespace else 'odom'
        self.base_frame = f"{self.namespace}/base_link" if self.namespace else 'base_link'

        self.__node = rclpy.create_node(
            'robot_driver',
            namespace=self.namespace,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )

        self.odom_publisher = self.__node.create_publisher(Odometry, 'odom', 10)
        self.joint_state_publisher = self.__node.create_publisher(JointState, 'joint_states', 10)
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__target_twist = Twist()
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        
        # 🚨 여기서 /clock 을 발행하지 않는다. 시계는 master 컨테이너의
        #    sim_clock_bridge 가 낸다 (04_UGV_SETUP.md 7절 ①).
        #
        #    예전에는 네임스페이스가 'ugv1' 일 때만 자기를 시계 마스터로 정했는데,
        #    브릿지를 도입한 뒤에도 이 코드가 남아 **발행자가 2개**가 됐다. 둘이
        #    미세하게 다른 시각을 쏘자 구독자 쪽에서 시각이 뒤로 갔고,
        #    tf2 가 "Detected jump back in time. Clearing TF buffer." 로 버퍼를
        #    계속 비워서 **Nav2 가 경로를 못 만들었다** (드론이 목표를 받고도
        #    cmd_vel_nav 를 0건 낸 원인. 실측).

        self.gps = self.__robot.getDevice('gps')
        if self.gps:
            self.gps.enable(self.__timestep)

        self.imu = self.__robot.getDevice('imu')
        if self.imu:
            self.imu.enable(self.__timestep)

        print(f"===== init() done for [{self.namespace}] "
              f"(바퀴 위치센서 {len(self.wheel_sensors)}/4) =====", flush=True)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __publish_joint_states(self, stamp):
        """바퀴 관절의 현재 각도를 발행한다.

        robot_state_publisher 가 이걸 받아야 바퀴 링크의 TF 를 만든다.
        연속 회전 관절이라 값이 계속 커지는데, TF 계산은 각도를 그대로 쓰므로 문제없다.
        """
        if not self.wheel_sensors:
            return

        msg = JointState()
        msg.header.stamp = stamp
        for joint_name, sensor in self.wheel_sensors.items():
            value = sensor.getValue()
            if math.isnan(value):
                continue
            msg.name.append(joint_name)
            msg.position.append(float(value))

        if msg.name:
            self.joint_state_publisher.publish(msg)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Webots 시간 가져오기
        wb_time = self.__robot.getTime()

        # 시뮬 시각. odom/TF/joint_states 가 모두 같은 시각을 써야 한다.
        curr_time = Time()
        curr_time.sec = int(wb_time)
        curr_time.nanosec = int((wb_time - int(wb_time)) * 1e9)

        vx = self.__target_twist.linear.x
        vy = self.__target_twist.linear.y
        wz = self.__target_twist.angular.z

        fl, fr, bl, br = mecanumControl(vx, vy, wz)

        self.bl_motor.setVelocity(fl)
        self.fl_motor.setVelocity(fr)
        self.br_motor.setVelocity(bl)
        self.fr_motor.setVelocity(br)

        self.__publish_joint_states(curr_time)

        if self.gps and self.imu:
            gps_vals = self.gps.getValues()

            if gps_vals and not math.isnan(gps_vals[0]):
                # 🌟 TF 브로드캐스트: 동적 프레임 적용 (odom -> base_link)
                t = TransformStamped()
                t.header.stamp = curr_time
                t.header.frame_id = self.odom_frame       
                t.child_frame_id = self.base_frame        

                t.transform.translation.x = float(gps_vals[0])
                t.transform.translation.y = float(gps_vals[1])
                t.transform.translation.z = float(gps_vals[2])

                rpy = self.imu.getRollPitchYaw()
                if rpy and not math.isnan(rpy[2]):
                    yaw = rpy[2]
                    t.transform.rotation.z = math.sin(yaw / 2.0)
                    t.transform.rotation.w = math.cos(yaw / 2.0)

                    # 🌟 Odometry 퍼블리시: 동적 프레임 적용
                    odom_msg = Odometry()
                    odom_msg.header.stamp = curr_time
                    odom_msg.header.frame_id = self.odom_frame     
                    odom_msg.child_frame_id = self.base_frame      

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