"""Mavic2ProMedium용 webots_ros2_driver 플러그인.

robot_driver.py(UGV)와 같은 형식이지만, 드론은 모터 4개로 6자유도를 다루는
underactuated 시스템이라 cmd_vel을 모터 속도로 바로 변환할 수 없다.
2단 구조로 처리한다.

  cmd_vel ──▶ [속도 외부 루프] ──▶ 자세 목표 ──▶ [자세/고도 내부 루프] ──▶ 모터 4개

내부 루프는 Webots 공식 mavic2pro 샘플의 제어 법칙을 그대로 옮긴 것이며,
게인 두 개만 다르다 (근거는 Readme 11-4 참고).

  k_rate_d      회전 관성이 제어 토크보다 빠르게 커져서(x28 vs x14) 상향
  k_vertical_d  순정은 고도를 P항만으로 제어해 준안정. 이 항이 없으면 39% 오버슈트

외부 루프는 순정에 없던 것으로, cmd_vel을 실제 m/s로 추종시킨다.
이것이 없으면 cmd_vel 값이 자세 오프셋이라는 임의 단위가 되어 Nav2가 못 쓴다.
동시에 정지 명령(0 m/s)을 능동적으로 유지하므로, 순정 제어 법칙의 고질적인
수평 드리프트(~0.09 m/s)도 사라진다.
"""

import math

import rclpy
from rclpy.parameter import Parameter

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# --- 내부 루프 (Webots 공식 샘플에서 이식) ---
K_VERTICAL_THRUST = 68.5   # 이 추력에서 기체가 부양한다
K_VERTICAL_OFFSET = 0.6    # 고도 안정화 기준 오프셋
K_VERTICAL_P = 3.0
K_VERTICAL_D = 8.0         # 수직 속도 D항 (순정에 없음)
K_ROLL_P = 50.0
K_PITCH_P = 30.0
K_RATE_D = 2.0             # 자이로 댐핑 (순정 대비 상향)

# --- 외부 루프 (cmd_vel 추종) ---
# P만 쓰면 정상상태 오차가 크다(1.0 m/s 명령에 0.59 m/s, 실측). 적분항이 이를 없앤다.
K_VEL_P = 2.0              # 속도 오차 -> 자세 외란
K_VEL_I = 1.0              # 적분항 (정상상태 오차 제거)
VEL_I_LIMIT = 4.0          # 적분 와인드업 상한
MAX_TILT_DISTURBANCE = 4.0 # 자세 외란 상한 (과도한 기울기 방지)
# 선회는 "목표 방위를 명령 각속도로 적분"하는 방식으로 처리한다.
# 각속도 오차에 P만 걸면 정상상태 오차가 크지만(0.5 rad/s 명령에 0.185 rad/s, 실측),
# 방위 오차 항이 각속도 오차의 적분 역할을 해서 오차가 사라진다.
# 덕분에 "유지"와 "선회"를 모드 분기 없이 같은 식으로 다룰 수 있다.
K_YAW_P = 6.0              # 방위 오차 -> 요 외란
K_YAW_RATE = 1.5           # 각속도 오차 (감쇠 항)
MAX_YAW_ERROR = 0.5        # 목표 방위가 실제보다 앞서 나가는 것을 제한 (안티 와인드업)

MOTOR_MAX_VELOCITY = 576.0 # PROTO의 RotationalMotor.maxVelocity
MIN_ALTITUDE = 0.3
MAX_ALTITUDE = 50.0
DEFAULT_ALTITUDE = 2.0


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    """각도를 [-pi, pi]로 정규화."""
    return math.atan2(math.sin(angle), math.cos(angle))


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class main:

    @staticmethod
    def __motor_range(motor):
        """모터의 가동 범위를 (min, max)로 돌려준다. 무제한이면 None.

        Webots 규약: minPosition == maxPosition == 0 이면 제한 없음.
        """
        lo, hi = motor.getMinPosition(), motor.getMaxPosition()
        if lo == 0.0 and hi == 0.0:
            return None
        return lo, hi

    @staticmethod
    def __clamp(value, limits):
        if limits is None:
            return value
        lo, hi = limits
        return min(max(value, lo), hi)

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__dt = self.__timestep / 1000.0

        # --- 프로펠러 모터: 속도 제어 모드 ---
        self.__motors = {}
        for key, name in (
            ('fl', 'front left propeller'),
            ('fr', 'front right propeller'),
            ('rl', 'rear left propeller'),
            ('rr', 'rear right propeller'),
        ):
            motor = self.__robot.getDevice(name)
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)
            self.__motors[key] = motor

        # --- 센서 ---
        self.__gps = self.__robot.getDevice('gps')
        self.__gps.enable(self.__timestep)
        self.__imu = self.__robot.getDevice('inertial unit')
        self.__imu.enable(self.__timestep)
        self.__gyro = self.__robot.getDevice('gyro')
        self.__gyro.enable(self.__timestep)

        # --- 짐벌 (각속도 댐핑, 순정과 동일) ---
        self.__camera_roll = self.__robot.getDevice('camera roll')
        self.__camera_pitch = self.__robot.getDevice('camera pitch')

        # 모터의 실제 가동 범위를 읽어 둔다. 짐벌 명령을 이 범위로 잘라 보내지 않으면
        # 기체가 크게 흔들릴 때 Webots 가 매 스텝 경고를 뱉는다:
        #   RotationalMotor "camera roll": too low requested position: -1.62 < -0.5
        # Webots 는 어차피 내부에서 자르므로 동작은 같지만, 콘솔이 경고로 도배되어
        # 진짜 경고를 못 보게 된다. PROTO 기준 roll [-0.5, 0.5], pitch [-0.5, 1.7].
        self.__gimbal_limits = {
            'roll': self.__motor_range(self.__camera_roll),
            'pitch': self.__motor_range(self.__camera_pitch),
        }

        if not rclpy.ok():
            rclpy.init(args=None)

        # 🌟 URDF에서 넘겨준 namespace 받기 (예: 'drone1')
        self.namespace = properties.get('namespace', '')
        self.odom_frame = f"{self.namespace}/odom" if self.namespace else 'odom'
        self.base_frame = f"{self.namespace}/base_link" if self.namespace else 'base_link'

        self.__node = rclpy.create_node(
            'drone_driver',
            namespace=self.namespace,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        self.odom_publisher = self.__node.create_publisher(Odometry, 'odom', 10)
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__target_twist = Twist()
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        # 목표 고도는 cmd_vel.linear.z로 적분해서 바꾼다
        self.target_altitude = DEFAULT_ALTITUDE
        # 목표 방위. 매 스텝 명령 각속도만큼 전진시킨다. None이면 첫 스텝에서 현재 방위로 초기화
        self.yaw_hold = None
        # 속도 루프 적분 상태 (기체 프레임)
        self.vel_i_x = 0.0
        self.vel_i_y = 0.0

        self.__node.get_logger().info(
            f"drone_driver 시작 [{self.namespace}] timestep={self.__timestep}ms")

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        wb_time = self.__robot.getTime()

        roll, pitch, yaw = self.__imu.getRollPitchYaw()
        roll_velocity, pitch_velocity, yaw_velocity = self.__gyro.getValues()
        position = self.__gps.getValues()
        velocity = self.__gps.getSpeedVector()

        if math.isnan(position[0]) or math.isnan(roll):
            return  # 첫 스텝에는 센서값이 아직 없다

        altitude = position[2]

        # ------------------------------------------------------------------
        # 짐벌 안정화 (각속도 댐핑)
        #
        # 명령을 모터 가동 범위로 자른다. 이 값은 각도가 아니라 **각속도에 비례**하는
        # 양이라, 기체가 급하게 흔들리면 쉽게 범위를 넘는다. 예를 들어 roll 계수가
        # 0.115 이므로 roll_velocity 가 4.3 rad/s 만 넘어도 한계(-0.5)를 벗어난다.
        # 소환 직후 낙하나 급기동에서 실제로 -1.62 까지 나왔다.
        # 자르지 않으면 Webots 가 매 스텝 "too low requested position" 경고를 뱉는다.
        # ------------------------------------------------------------------
        self.__camera_roll.setPosition(
            self.__clamp(-0.115 * roll_velocity, self.__gimbal_limits['roll']))
        self.__camera_pitch.setPosition(
            self.__clamp(-0.1 * pitch_velocity, self.__gimbal_limits['pitch']))

        # ------------------------------------------------------------------
        # 외부 루프: cmd_vel(m/s) -> 자세 외란
        # ------------------------------------------------------------------
        target_vx = self.__target_twist.linear.x
        target_vy = self.__target_twist.linear.y
        target_wz = self.__target_twist.angular.z

        # 목표 고도 갱신 (linear.z는 상승 속도 명령)
        self.target_altitude = clamp(
            self.target_altitude + self.__target_twist.linear.z * self.__dt,
            MIN_ALTITUDE, MAX_ALTITUDE)

        # 월드 프레임 속도를 기체 프레임으로 회전
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        vx_body = cos_yaw * velocity[0] + sin_yaw * velocity[1]
        vy_body = -sin_yaw * velocity[0] + cos_yaw * velocity[1]

        err_vx = target_vx - vx_body
        err_vy = target_vy - vy_body
        self.vel_i_x = clamp(self.vel_i_x + err_vx * self.__dt, -VEL_I_LIMIT, VEL_I_LIMIT)
        self.vel_i_y = clamp(self.vel_i_y + err_vy * self.__dt, -VEL_I_LIMIT, VEL_I_LIMIT)

        # 전진하려면 기수를 내려야 하므로(pitch 음수) 부호가 반대
        pitch_disturbance = clamp(-(K_VEL_P * err_vx + K_VEL_I * self.vel_i_x),
                                  -MAX_TILT_DISTURBANCE, MAX_TILT_DISTURBANCE)
        roll_disturbance = clamp(K_VEL_P * err_vy + K_VEL_I * self.vel_i_y,
                                 -MAX_TILT_DISTURBANCE, MAX_TILT_DISTURBANCE)

        # 선회: 목표 방위를 명령 각속도로 적분한다.
        # target_wz가 0이면 목표 방위가 그대로 유지되므로 자동으로 "방위 유지"가 된다.
        if self.yaw_hold is None:
            self.yaw_hold = yaw
        self.yaw_hold = wrap_angle(self.yaw_hold + target_wz * self.__dt)
        yaw_error = clamp(wrap_angle(self.yaw_hold - yaw), -MAX_YAW_ERROR, MAX_YAW_ERROR)
        self.yaw_hold = wrap_angle(yaw + yaw_error)  # 목표가 달아나지 않게 되감기
        yaw_disturbance = clamp(
            K_YAW_P * yaw_error + K_YAW_RATE * (target_wz - yaw_velocity),
            -MAX_TILT_DISTURBANCE, MAX_TILT_DISTURBANCE)

        # ------------------------------------------------------------------
        # 내부 루프: 자세/고도 안정화 -> 모터 4개
        # ------------------------------------------------------------------
        roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + K_RATE_D * roll_velocity + roll_disturbance
        pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + K_RATE_D * pitch_velocity + pitch_disturbance
        yaw_input = yaw_disturbance

        clamped_difference_altitude = clamp(
            self.target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
        vertical_input = (K_VERTICAL_P * clamped_difference_altitude ** 3
                          - K_VERTICAL_D * velocity[2])

        base = K_VERTICAL_THRUST + vertical_input
        fl = base - roll_input + pitch_input - yaw_input
        fr = base + roll_input + pitch_input + yaw_input
        rl = base - roll_input - pitch_input + yaw_input
        rr = base + roll_input - pitch_input - yaw_input

        self.__motors['fl'].setVelocity(clamp(fl, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY))
        self.__motors['fr'].setVelocity(clamp(-fr, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY))
        self.__motors['rl'].setVelocity(clamp(-rl, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY))
        self.__motors['rr'].setVelocity(clamp(rr, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY))

        # ------------------------------------------------------------------
        # odom / TF 발행 (UGV와 달리 z와 roll/pitch까지 싣는다)
        # ------------------------------------------------------------------
        curr_time = Time()
        curr_time.sec = int(wb_time)
        curr_time.nanosec = int((wb_time - int(wb_time)) * 1e9)

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp = curr_time
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.__tf_broadcaster.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header.stamp = curr_time
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = t.transform.translation.x
        odom_msg.pose.pose.position.y = t.transform.translation.y
        odom_msg.pose.pose.position.z = t.transform.translation.z
        odom_msg.pose.pose.orientation = t.transform.rotation
        odom_msg.twist.twist.linear.x = float(vx_body)
        odom_msg.twist.twist.linear.y = float(vy_body)
        odom_msg.twist.twist.linear.z = float(velocity[2])
        odom_msg.twist.twist.angular.z = float(yaw_velocity)
        self.odom_publisher.publish(odom_msg)
