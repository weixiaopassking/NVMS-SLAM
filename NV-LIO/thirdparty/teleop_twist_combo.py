#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# 键位映射
move_bindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    'q': (1, 1),
    'e': (1, -1),
    'z': (-1, 1),
    'c': (-1, -1),
}

DEFAULT_LINEAR_SPEED = 0.3
DEFAULT_ANGULAR_SPEED = 1.0

def getKey():
    try:
        tty.setcbreak(sys.stdin.fileno())  # 更适合中断
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            return sys.stdin.read(1)
        return ''
    except:
        return ''

def print_status(lin, ang):
    print("\r当前线速度: {:.2f} m/s, 角速度: {:.2f} rad/s      ".format(lin, ang), end='')

def main():
    settings = termios.tcgetattr(sys.stdin)  # 备份终端设置
    rospy.init_node('teleop_twist_combo')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    linear_speed = DEFAULT_LINEAR_SPEED
    angular_speed = DEFAULT_ANGULAR_SPEED
    twist = Twist()

    print("""
🎮 高级遥控器 - 支持速度调节 / 丝滑停止 / Ctrl+C
------------------------------------------------
方向控制（可组合）：
    q    w    e
    a    s    d
    z         c

速度调节：
  i / k : 增/减线速度
  l / j : 增/减角速度
  0     : 重置速度
空格键  ：急停
CTRL+C ：退出
""")

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == '\x03':  # Ctrl+C ASCII
                break
            elif key in move_bindings:
                lin, ang = move_bindings[key]
                twist.linear.x = linear_speed * lin
                twist.angular.z = angular_speed * ang
            elif key == ' ':
                twist = Twist()
            elif key == 'i':
                linear_speed += 0.05
                print_status(linear_speed, angular_speed)
                continue
            elif key == 'k':
                linear_speed = max(0.0, linear_speed - 0.05)
                print_status(linear_speed, angular_speed)
                continue
            elif key == 'l':
                angular_speed += 0.1
                print_status(linear_speed, angular_speed)
                continue
            elif key == 'j':
                angular_speed = max(0.0, angular_speed - 0.1)
                print_status(linear_speed, angular_speed)
                continue
            elif key == '0':
                linear_speed = DEFAULT_LINEAR_SPEED
                angular_speed = DEFAULT_ANGULAR_SPEED
                print_status(linear_speed, angular_speed)
                continue
            else:
                continue

            pub.publish(twist)

    except KeyboardInterrupt:
        pass
    finally:
        # 确保停止小车 + 恢复终端
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n🛑 控制退出，终端已恢复。")

if __name__ == '__main__':
    main()
