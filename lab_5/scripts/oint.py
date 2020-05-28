#!/usr/bin/env python

import rospy
from lab_4.srv import OintControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from lab_4.oint import OintInterpolator

class OintInt2(OintInterpolator):
    def __init__(self, frequency,  pose_pub, path_pub, start_pose=[0 for x in range(7)]):
        super(OintInt2, self).__init__(frequency,  pose_pub, path_pub, start_pose=[0 for x in range(7)])

    def interpolation_callback(self, msg):
        head_pose = rospy.wait_for_message('/head_pose', PoseStamped, timeout=1)
        p = head_pose.pose.position
        self.previous_pose = [p.x, p.y, p.z, 0, 0, 0, 1]
        super(OintInt2, self).interpolation_callback(msg)

if __name__ == "__main__":
    rospy.init_node('oint_node')
    pose_pub = rospy.Publisher('/oint_pose', PoseStamped, queue_size=1)
    path_pub = rospy.Publisher('/oint_path', Path, queue_size=1)

    head_pose = rospy.wait_for_message('/head_pose', PoseStamped, timeout=3)
    p = head_pose.pose.position
    start_position = [p.x, p.y, p.z, 0, 0, 0, 1]
    interpolator = OintInt2(100, pose_pub, path_pub, start_pose=start_position)
    service = rospy.Service('oint_control_srv', OintControl, interpolator.interpolation_callback)
    rospy.loginfo("Interpolator ready")
    rospy.spin()
