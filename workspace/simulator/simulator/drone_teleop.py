"""드론용 키보드 텔레옵.

simulator/keyboard.py(UGV용)와 같은 방식이지만 드론에 맞게 축을 바꿨다.
UGV에 없는 고도(linear.z)가 있고, 좌우가 조향이 아니라 평행이동이다.

실행:
    ros2 run simulator drone_teleop --ros-args -r __ns:=/drone1
"""

import sys

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
드론 키보드 조종

Publish topic: cmd_vel  (geometry_msgs/msg/Twist)

---------------------------
        W : 전진
   A         D : 좌 / 우 평행이동
        S : 후진

   Q / E : 좌 / 우 선회
   R / F : 상승 / 하강

   Space : 정지 (제자리 호버)
   = / - : 속도 증가 / 감소
   Ctrl-C : 종료
---------------------------
주의: 정지는 "속도 0"이지 "위치 고정"이 아니다.
      고도는 유지되지만 수평 위치는 멈춘 자리에 머문다.
"""

# key: (x, y, z, yaw)
moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 1, 0, 0),    # +y = 좌 (FLU)
    'd': (0, -1, 0, 0),
    'q': (0, 0, 0, 1),    # +yaw = 좌선회
    'e': (0, 0, 0, -1),
    'r': (0, 0, 1, 0),    # 상승
    'f': (0, 0, -1, 0),
    ' ': (0, 0, 0, 0),    # 호버
}

speedBindings = {
    '=': (1.1, 1.1),
    '-': (.9, .9),
}


def getKey(settings):
    if sys.platform == 'win32':
        return msvcrt.getwch()
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()
    rclpy.init()

    node = rclpy.create_node('drone_teleop')
    pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

    speed = 0.5   # m/s
    turn = 0.5    # rad/s
    x = y = z = th = 0.0

    try:
        print(msg)
        print(f'속도 {speed:.2f} m/s\t선회 {turn:.2f} rad/s')
        while True:
            key = getKey(settings)
            if key in moveBindings:
                x, y, z, th = (float(v) for v in moveBindings[key])
            elif key in speedBindings:
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(f'속도 {speed:.2f} m/s\t선회 {turn:.2f} rad/s')
                continue
            else:
                x = y = z = th = 0.0
                if key == '\x03':  # Ctrl-C
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # 종료 시 반드시 정지 명령을 보낸다 (안 보내면 마지막 속도로 계속 난다)
        pub.publish(geometry_msgs.msg.Twist())
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
