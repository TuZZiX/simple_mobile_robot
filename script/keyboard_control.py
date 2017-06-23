# !/usr/bin/env python

import curses

import rospy
from geometry_msgs.msg import Twist

speedBindings = {
    'a': (1.1, 1.1),
    's': (.9, .9),
    'd': (1.1, 1),
    'w': (.9, 1),
    'z': (0, 0),
}

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def main():

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    th = 0

    try:
        print vels(speed, turn)
        key = ""
        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(1)
        stdscr.clear()
        stdscr.addstr("Detected key:")
        while 1:
            key = stdscr.getkey()
            stdscr.clear()
            stdscr.addstr("Detected key:")
            stdscr.addstr(str(key))
            if key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print vels(speed, turn)
            else:
                x = 0
                th = 0
                if key == '\x03':
                    break
            twist = Twist()
            twist.linear.x = x * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

if __name__ == "__main__":
    main()


