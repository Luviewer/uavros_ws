#!/usr/bin/env python3
# coding: utf-8

import sys
import rospy
import curses
from geometry_msgs.msg import Twist
from jethexa_controller_interfaces.msg import Traveling


def main(stdscr):
    msg = Twist()
    last_keycode = ord('.')
    while not rospy.is_shutdown():
        keycode = stdscr.getch()
        if keycode == ord('\n'):
            stdscr.addstr(0, 0, "立正")
            tmsg = Traveling()
            tmsg.gait = -2
            tmsg.time = 1.0
            traveling_pub.publish(tmsg)
        if last_keycode == keycode:
            continue
        last_keycode = keycode

        if keycode == curses.KEY_UP or keycode == ord('w'):
            stdscr.addstr(0, 0, "前进")
            msg.linear.x, msg.linear.y = 0.08, 0.0
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        elif keycode == curses.KEY_DOWN or keycode == ord('s'):
            stdscr.addstr(0, 0, "后退")
            msg.linear.x, msg.linear.y = -0.08, 0.0
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        elif keycode == curses.KEY_LEFT:
            stdscr.addstr(0, 0, "左转")
            msg.linear.x, msg.linear.y = 0.0, 0.0
            msg.angular.z = 0.25
            cmd_vel_pub.publish(msg)

        elif keycode == curses.KEY_RIGHT:
            stdscr.addstr(0, 0, "右转")
            msg.linear.x, msg.linear.y = 0.0, 0.0
            msg.angular.z = -0.25
            cmd_vel_pub.publish(msg)

        elif keycode == ord('a'):
            stdscr.addstr(0, 0, "左移")
            msg.linear.x, msg.linear.y = 0.0, 0.08
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        elif keycode == ord('d'):
            stdscr.addstr(0, 0, "右移")
            msg.linear.x, msg.linear.y = 0.0, -0.08
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        elif keycode == ord(' '):
            stdscr.addstr(0, 0, "停止")
            msg.linear.x, msg.linear.y = 0.0, 0.0
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        else:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('keyboard_cotnrol_node')
        topic_prefix = rospy.get_param("~topic_prefix", "jethexa_controller")
        cmd_vel_pub = rospy.Publisher(topic_prefix + '/cmd_vel', Twist, queue_size=1)
        traveling_pub = rospy.Publisher(topic_prefix + '/traveling', Traveling, queue_size=1)
        curses.wrapper(main)
        node.loop()
    except Exception as e:
        rospy.logerr(str(e))

