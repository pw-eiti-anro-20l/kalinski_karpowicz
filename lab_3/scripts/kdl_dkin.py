#!/usr/bin/python

import rospy
import yaml
import os
from PyKDL import JntArray, Frame, Segment, Joint, Chain, ChainFkSolverPos_recursive
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class KdlDkin:
    def __init__(self, params, publisher):
        self.pose_publisher = publisher
        self.params = params
        self.last_correct_pose = None
        self.constraints = self.read_constraints(os.path.dirname(os.path.realpath(__file__))+ '/../config/joint_constraints.yaml')

    def read_constraints(self, path):
        with open(path, 'r') as f:
            constraints = yaml.load(f)
            print(constraints)
            return [{'min':constraints[c][0], 'max':constraints[c][1]} for c in constraints]

    def check_constraints(self, msg):
        for rot,limit in zip(msg.position, self.constraints):
            if(rot > limit['max'] or rot < limit['min']):
                return False
        return True

    def joint_state_callback(self, msg):
        positions = JntArray(3)
        chain =Chain()   
        kdl_frame = Frame()

        d = 0
        theta = 0

        order = [1, 2, 0]

        for i in order:
            joint = self.params[i]
            a = joint["a"]
            d = joint["d"]
            alpha = joint["alpha"]
            theta = joint["theta"]

            frame = kdl_frame.DH(a, alpha, d, theta)
            chain.addSegment(Segment(Joint(Joint.RotZ), frame))
            positions[i] = msg.position[i]

        fk_solver = ChainFkSolverPos_recursive(chain)
        result = Frame()
        fk_solver.JntToCart(positions, result)

        kdl_pose = PoseStamped()
        kdl_pose.header.frame_id = 'base_link'
        kdl_pose.header.stamp = rospy.Time.now()

        kdl_pose.pose.position.x = result.p[0]
        kdl_pose.pose.position.y = result.p[1]
        kdl_pose.pose.position.z = result.p[2]
        quat = result.M.GetQuaternion()
        kdl_pose.pose.orientation.x = quat[0]
        kdl_pose.pose.orientation.y = quat[1]
        kdl_pose.pose.orientation.z = quat[2]
        kdl_pose.pose.orientation.w = quat[3]

        if(self.check_constraints(msg)):
            self.pose_publisher.publish(kdl_pose)
            self.last_correct_pose = kdl_pose
        else:
            rospy.logerr('Joint state out of our constraints!')
            self.pose_publisher.publish(self.last_correct_pose)

if __name__ == '__main__':
    rospy.init_node('kdl_dkin', anonymous = False)
    params = rospy.get_param('robot_params')
    for joint in params:
        a, d, alpha, _ = [joint[x] for x in joint]
        print(str(a)+ '  ' + str(d) + '  ' + str(alpha))

    pose_pub = rospy.Publisher('head_pose', PoseStamped, queue_size=1)
    kdl_dkin = KdlDkin(params, pose_pub)
    rospy.Subscriber("joint_states", JointState, kdl_dkin.joint_state_callback)
    rospy.spin()