#!/usr/bin/env python
import rospy
import os
from pynput import keyboard
from geometry_msgs.msg import Twist


class Keyboard_monitor:
    def __init__(self, cmd_pub, front_key, back_key, left_key, right_key):

        self.front_key = front_key
        self.back_key = back_key
        self.left_key = left_key
        self.right_key = right_key

        self.default_speed = 1

        self.cmd_msg = Twist()
        self.msg_values = {front_key:0, back_key:0, left_key:0, right_key:0}
   
    
    def on_press(self, key):
        try:
            self.msg_values[key.char] = self.default_speed
        except:
            pass
    
    def on_release(self, key):
        try:
            self.msg_values[key.char] = 0
        except:
            pass
    
    def publish(self):
        if(self.msg_values[self.back_key] == 1):
            self.cmd_msg.linear.x = -self.default_speed
        else:
            self.cmd_msg.linear.x = self.default_speed

        if(self.msg_values[self.left_key] == 1):
            self.cmd_msg.angular.z = self.default_speed
        else:
            self.cmd_msg.angular.z = -self.default_speed

        cmd_pub.publish(self.cmd_msg)


if __name__ == '__main__':
    rospy.init_node('keyboard_steering', anonymous=True)

# get node params
    default_speed = rospy.get_param('~default_speed', 1)
    publish_rate = rospy.get_param('~publish_rate', 10)
    cmd_topic = rospy.get_param('~cmd_topic', '/turtle1/cmd_vel')
    publish_rate = rospy.get_param('~publish_rate', 10)

# get steering keys params
    f_key = rospy.get_param('~front_key', 'w')
    b_key = rospy.get_param('~back_key', 's')
    l_key = rospy.get_param('~left_key', 'a')
    r_key = rospy.get_param('~right_key', 'd')

# print loaded params
    rospy.loginfo('default_steering_speed = ' + str(default_speed))
    rospy.loginfo('sim_controls_publish_rate = ' + str(publish_rate))
    rospy.loginfo('cmd_topic = ' + str(cmd_topic))
    rospy.loginfo('publish_rate = ' + str(publish_rate))

# set publisher and subscribers
    cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)

#init monitor
    monitor = Keyboard_monitor(cmd_pub, f_key, b_key, l_key, r_key)

# init keyboard listener
    listener = keyboard.Listener(
                on_press=monitor.on_press,
                on_release=monitor.on_release)
    listener.start()
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        monitor.publish()
        rate.sleep()

    rospy.signal_shutdown("manually closed")