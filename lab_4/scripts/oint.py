#!/usr/bin/env python

import rospy
from lab_4.srv import OintControl
from geometry_msgs.msg import PoseStamped
import math
from nav_msgs.msg import Path
from jint import GenericInterpolator


class OintInterpolator(GenericInterpolator):
    def __init__(self, frequency,  pose_pub, path_pub):
        super(OintInterpolator, self).__init__()
        self.pose_pub = pose_pub
        self.path_pub = path_pub
        self.path = Path()
        self.frequency = frequency
        self.previous_pose = [0 for x in range(7)]
        self.previous_pose[6] = 1

    def interpolation_allowed(self, req):
        if req.execution_time <= float(0):
            rospy.logerr("Execution time less than 0")
            return False
        elif req.interpolation_type != "linear" and req.interpolation_type != "quadratic":
            rospy.logerr("Wrong interpolation type. (available types: linear, quadratic)")
            return False
        return True

    def interpolation_callback(self, req):

        if not self.interpolation_allowed(req):
            return False

        new_pos = [req.x, req.y, req.z, req.qx, req.qy, req.qz, req.qw]
        

        rate = rospy.Rate(self.frequency)

        current_time = 0.
        frames_number = int(math.ceil(req.execution_time * self.frequency))

        
        if req.interpolation_type == "linear":

            for i in range(frames_number + 1):
                position = []
                for k in range(0, 7):
                    position.append(self.linear_interpolation(self.previous_pose[k], new_pos[k], req.execution_time, current_time))

                pose_stamped = self.get_pose_from_position(position)
                self.pose_pub.publish(pose_stamped)
                self.publish_path(pose_stamped)
                current_time = current_time + 1. / self.frequency
                rate.sleep()

        else:
            coeffs = self.quadratic_coeffs(self.previous_pose, new_pos, req.execution_time)
            for i in range(frames_number + 1):
                position = []
                for k in range(0, 7):
                    position.append(self.quadratic_interpolation(self.previous_pose[k], new_pos[k], req.execution_time, current_time, coeffs[k]))

                pose = self.get_pose_from_position(position)
                self.pose_pub.publish(pose)
                self.publish_path(pose)
                current_time = current_time + 1. / self.frequency
                rate.sleep()

        self.previous_pose = new_pos
        rospy.loginfo('request handled')
        return True
    
    def get_pose_from_position(self, position):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'base_link'
        p.pose.position.x = position[0]
        p.pose.position.y = position[1]
        p.pose.position.z = position[2]
        p.pose.orientation.x = position[3]
        p.pose.orientation.y = position[4]
        p.pose.orientation.z = position[5]
        p.pose.orientation.w = position[6]
        return p
    
    def publish_path(self, pose):
	    self.path.header = pose.header
	    self.path.poses.append(pose)
	    self.path_pub.publish(self.path)


if __name__ == "__main__":
    rospy.init_node('oint_node')
    pose_pub = rospy.Publisher('/oint_pose', PoseStamped, queue_size=1)
    path_pub = rospy.Publisher('/oint_path', Path, queue_size=1)

    interpolator = OintInterpolator(10, pose_pub, path_pub)
    service = rospy.Service('oint_control_srv', OintControl, interpolator.interpolation_callback)
    rospy.loginfo("Interpolator ready")
    rospy.spin()
