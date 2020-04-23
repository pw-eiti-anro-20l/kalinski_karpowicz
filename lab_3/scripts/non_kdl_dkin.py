#!/usr/bin/python

import json
import rospy
import rospkg
from sensor_msgs.msg import JointState
from tf.transformations import *
from geometry_msgs.msg import PoseStamped
import yaml
import os

class NonKdlDkin:

    def __init__(self, params, publisher):
        self.params = params
        self.pose_publisher = publisher
        self.constraints = None
        self.last_correct_pose = None
    
    def read_constraints(self, path):
        with open(path, 'r') as f:
            constraints = yaml.load(f)
            print(constraints)
            self.constraints = [{'min':constraints[c][0], 'max':constraints[c][1]} for c in constraints]
            print(self.constraints)

    def check_constraints(self, msg):
        for rot,limit in zip(msg.position,self.constraints):
            if(rot > limit['max'] or rot < limit['min']):
                return False
        return True


    def joint_state_callback(self, msg):
        X, Z = (1, 0, 0), (0, 0, 1)
        result = translation_matrix((0,0,0))
    
        for i, joint in zip(range(1,4), self.params):
            a = joint['a']
            d = joint['d']
            alpha = joint['alpha']
            x_transl = translation_matrix((a, 0, 0))
            x_rot = rotation_matrix(alpha, X)
            z_transl = translation_matrix((0, 0, d))
            z_rot = rotation_matrix(msg.position[i-1], Z)

            transformation = concatenate_matrices(x_rot, x_transl, z_rot, z_transl)
            result = concatenate_matrices(result, transformation)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "base_link"

        xyz_mat = translation_from_matrix(result)
        pose.pose.position.x = xyz_mat[0]
        pose.pose.position.y = xyz_mat[1]
        pose.pose.position.z = xyz_mat[2]

        xyzw_mat = quaternion_from_matrix(result)
        pose.pose.orientation.x = xyzw_mat[0]
        pose.pose.orientation.y = xyzw_mat[1]
        pose.pose.orientation.z = xyzw_mat[2]
        pose.pose.orientation.w = xyzw_mat[3]
        if(self.check_constraints(msg)):
            self.pose_publisher.publish(pose)
            self.last_correct_pose = pose
        else:
            rospy.logerr('Joint state out of our constraints!')
            self.pose_publisher.publish(self.last_correct_pose)


if __name__ == '__main__':
    rospy.init_node('non_kdl_dkin', anonymous = False)
    params = rospy.get_param('robot_params')
    for joint in params:
        a, d, alpha, _ = [joint[x] for x in joint]
        print(str(a)+ '  ' + str(d) + '  ' + str(alpha))

    pose_pub = rospy.Publisher('head_pose', PoseStamped, queue_size=1)
    non_kdl_dkin = NonKdlDkin(params, pose_pub)
    rospy.Subscriber("joint_states", JointState, non_kdl_dkin.joint_state_callback)
    constraints_file = os.path.dirname(os.path.realpath(__file__))
    non_kdl_dkin.read_constraints(constraints_file+ '/../config/joint_constraints.yaml')
    rospy.spin()

