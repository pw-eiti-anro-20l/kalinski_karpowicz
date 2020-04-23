#!/usr/bin/python

import json
import rospy
import rospkg
from sensor_msgs.msg import JointState
from tf.transformations import *
from geometry_msgs.msg import PoseStamped

class NonKdlDkin:

    def __init__(self, params, publisher):
        self.params = params
        self.pose_publisher = publisher

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

        poseStamped = PoseStamped()
        poseStamped.header.stamp = msg.header.stamp
        poseStamped.header.frame_id = "base_link"

        xyz_mat = translation_from_matrix(result)
        poseStamped.pose.position.x = xyz_mat[0]
        poseStamped.pose.position.y = xyz_mat[1]
        poseStamped.pose.position.z = xyz_mat[2]

        xyzw_mat = quaternion_from_matrix(result)
        poseStamped.pose.orientation.x = xyzw_mat[0]
        poseStamped.pose.orientation.y = xyzw_mat[1]
        poseStamped.pose.orientation.z = xyzw_mat[2]
        poseStamped.pose.orientation.w = xyzw_mat[3]

        self.pose_publisher.publish(poseStamped)




if __name__ == '__main__':
    rospy.init_node('non_kdl_dkin', anonymous = False)
    params = rospy.get_param('robot_params')
    for joint in params:
        a, d, alpha, _ = [joint[x] for x in joint]
        print(str(a)+ '  ' + str(d) + '  ' + str(alpha))

    pose_pub = rospy.Publisher('head_pose_non_kdl', PoseStamped, queue_size=1)
    non_kdl = NonKdlDkin(params, pose_pub)
    rospy.Subscriber("joint_states", JointState, non_kdl.joint_state_callback)
    rospy.spin()

