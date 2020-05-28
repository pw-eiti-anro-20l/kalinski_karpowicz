#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from math import atan2, sqrt, pi
from lab_3.kdl_dkin import KdlDkin

# KdlDkin reads params, and constraints,
# publisher argument in super() constructor is None, couse we don't need this publisher here
class Ikin(KdlDkin):
    def __init__(self, params, state_publisher):
        super(Ikin, self).__init__(params, None)
        self.state_publisher = state_publisher
        self.current_theta = [0, 0, 0]
        self.a2 = 0.5 #
        self.a3 = 0.2 #tool length
    
    def pose_callback(self, msg):
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        if px**2 + py**2 <= 0.:
            rospy.logerr('Infinite solutions')
            return

        theta = [None]*3

        c3 = (px**2 + py**2 + pz**2 - self.a2**2 - self.a3**2)/(2*self.a2*self.a3)
        try:
            s3 = sqrt(1 - c3**2)
        except ValueError:
            rospy.logerr('Position unreachable')
            return

        flag = False

        #Joint_1 (base_to_link_1)
        if abs(atan2(py, px) - self.current_theta[0]) < abs((atan2(-py, -px)) - self.current_theta[0]):
            theta[0] = atan2(py, px)
            flag = True
        else:
            theta[0] = atan2(-py, -px)

        #Joint_2 (link1_to_link_2) and Joint_3 (link2_to_link_3)
        if abs(atan2(s3, c3) - self.current_theta[2]) < abs(atan2(-s3, c3) - self.current_theta[2]):
            theta[2] = atan2(s3, c3)
            if flag:
                theta[1] = atan2(-pz, sqrt(px**2 + py**2)) - atan2(self.a3*s3, self.a2 + self.a3*c3)
            else:
                theta[1] = pi - atan2(-pz, sqrt(px**2 + py**2)) + atan2(self.a3*s3, self.a2 + self.a3*c3)
        else:
            theta[2] = atan2(-s3, c3)
            if flag:
                theta[1] = atan2(-pz, sqrt(px**2 + py**2)) - atan2(-self.a3*s3, self.a2 + self.a3*c3)
            else:
                theta[1] = pi - atan2(-pz, sqrt(px**2 + py**2)) + atan2(-self.a3*s3, self.a2 + self.a3*c3)

        
        if not (self.check_constraints(theta)):
            rospy.logerr('Joint out of constraints')
        self.current_theta = theta
        rospy.loginfo(theta)

        jointState = self.create_joint_state_msg(theta)
        self.state_publisher.publish(jointState)

    def create_joint_state_msg(self, theta):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['link_0_to_link_1', 'link_1_to_link_2', 'link_2_to_link_3']
        joint_state.position = theta
        joint_state.velocity = []
        joint_state.effort = []
        return joint_state

    def check_constraints(self, theta):
        for i in range(3):
            if theta[i] < self.constraints[i]['min'] or theta[i] > self.constraints[i]['max']:
                return False
        return True

if __name__ == "__main__":
    rospy.init_node('ikin_node', anonymous = False)
    params = rospy.get_param('robot_params')

    state_publisher = rospy.Publisher("/joint_state", JointState, queue_size=1)
    ikin = Ikin(params, state_publisher)
    rospy.Subscriber("/oint_pose", PoseStamped, ikin.pose_callback)
    rospy.spin()