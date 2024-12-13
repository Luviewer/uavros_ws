#!/usr/bin/env python3

import sys
from enum import Enum
import time
import math
import rospy
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
from jethexa_sdk import misc, buzzer
from jethexa_controller import client


AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''


class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3


class JoystickControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.node_name = name
        self.jethexa = client.Client(self)
        self.last_axes =  dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.gait = 1 # 步态
        self.joy_sub = rospy.Subscriber('joy', sensor_msg.Joy, self.joy_callback) 
        self.transform = [0, 0, 0], [0, 0, 0]
        self.do_transform = True
        self.v_x = 0
        self.v_y = 0

    def axes_callback(self, axes):
        pitch, yaw, duration = 0.0, 0.0, 0.02
        pitch = 0.05 if axes['ly'] > 0.5 else -0.05 if axes['ly'] < -0.5 else 0
        yaw = 0.05 if axes['lx'] > 0.5 else -0.05 if axes['lx'] < -0.5 else 0
        if pitch != 0 or yaw != 0:
            self.jethexa.set_head_relatively(pitch=pitch, yaw=yaw, duration=duration)
        pitch = 0.04 if axes['ry'] > 0.5 else -0.04 if axes['ry'] < -0.5 else 0
        roll = 0.04 if axes['rx'] > 0.5 else -0.04 if axes['rx'] < -0.5 else 0
        if pitch != 0 or roll != 0:
            self.transform[1][0] = roll
            self.transform[1][1] = pitch
            self.do_transform = True


    def select_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait += 1
            if self.gait >= 3:
                self.gait = 1
            for i in range(self.gait):
                buzzer.on()
                rospy.sleep(0.1)
                buzzer.off()
                rospy.sleep(0.1)

    def l1_callback(self, new_state):
        if new_state == ButtonState.Holding:
            self.transform[0][2] = 4
            self.do_transform = True

    def l2_callback(self, new_state):
        if new_state == ButtonState.Holding:
            self.transform[0][2] = -4
            self.do_transform = True

    def r1_callback(self, new_state):
        if self.mode == 0:
            pass
        else:
            pass

    def r2_callback(self, new_state):
        if self.mode == 0:
            pass
        else:
            pass

    def square_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        if self.mode == 0:
            pass
        else:
            pass

    def circle_callback(self, new_state):
        if self.mode == 0:
            pass
        else:
            pass

    def triangle_callback(self, new_state):
        pass

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.jethexa.traveling(gait=-2, time=1, steps=0)
        
    def hat_xl_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # self.jethexa.traveling(gait=self.gait, stride=0, height=20,  rotation=math.radians(10), steps=0, time=0.6)
            self.jethexa.cmd_vel(0.0, 0.0, math.radians(5))
        elif new_state == ButtonState.Released:
            self.jethexa.cmd_vel(0.0, 0.0, 0.0)
            # self.jethexa.traveling(gait=0, time=1, steps=0)
        else:
            pass

    def hat_xr_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # self.jethexa.traveling(gait=self.gait, stride=0, height=20, rotation=math.radians(-10), steps=0, time=0.6)
            self.jethexa.cmd_vel(0.0, 0.0, math.radians(-5))
        elif new_state == ButtonState.Released:
            # self.jethexa.traveling(gait=0, time=1, steps=0)
            self.jethexa.cmd_vel(0.0, 0.0, 0.0)
        else:
            pass

    def hat_yu_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # self.jethexa.traveling(gait=self.gait, stride=30, direction=0, height=10, steps=0, time=3.0)
            self.jethexa.cmd_vel(0.05, 0.0, 0.0)
        elif new_state == ButtonState.Released:
            self.jethexa.cmd_vel(0.0, 0.0, 0.0)
            # self.jethexa.traveling(gait=0, time=1, steps=0)
        else:
            pass

    def hat_yd_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # self.jethexa.traveling(gait=self.gait, stride=30, height=10, direction=math.radians(180), steps=0, time=3.0)
            self.jethexa.cmd_vel(-0.05, 0.0, 0.0)
        elif new_state == ButtonState.Released:
            self.jethexa.cmd_vel(-0.0, 0.0, 0.0)
            # self.jethexa.traveling(gait=0, time=1, steps=0)
        else:
            pass

    def joy_callback(self, joy_msg: sensor_msg.Joy):
        self.transform = [0, 0, 0], [0, 0, 0]
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        # 方向帽的 ad 值转为 bool 值
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))
        #
        #for key, value in axes.items(): # 轴的值被改变
        #    if self.last_axes[key] != value:
        #        axes_changed = True
        #if axes_changed:
        try:
            self.axes_callback(axes)
        except Exception as e:
            rospy.logerr(str(e))
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                # rospy.loginfo(key + ': ' + str(new_state))
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(str(e))
        if self.do_transform:
            self.jethexa.pose_transform_euler(self.transform[0], self.transform[1], 0.1)
            self.do_transform = False
        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickControlNode('joystick_control')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

