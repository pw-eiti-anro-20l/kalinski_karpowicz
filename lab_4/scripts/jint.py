#! /usr/bin/python

import sys
import rospy
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from lab_4v0.srv import JintControl
from lab_3.kdl_dkin import KdlDkin

class GenericInterpolator(object):
    def __init__(self):
        pass

    def linear_interpolation(self, start_angle, end_angle, execution_time, current_time):
        return (start_angle + (float(end_angle - start_angle) / execution_time) * current_time)

    def quadratic_coeffs(self, start_pos, request_pos, execution_time):
        coeffs = []
        for i in range(0, len(start_pos)):
            coeffs.append(2. * float(request_pos[i] - start_pos[i]) / execution_time ** 2)
        return coeffs

    def quadratic_interpolation(self, start_angle, end_angle, execution_time, current_time, coeff):
        if current_time < execution_time / 2.:
            return start_angle + coeff * current_time ** 2
        else:
            return end_angle - coeff * (execution_time - current_time) ** 2


class JintInterpolator(KdlDkin, GenericInterpolator):
	def __init__(self, frequency, params, path_pub, pose_pub, head_pub):
		super(JintInterpolator, self).__init__(params, pose_pub)
		self.frequency = frequency
		self.path = Path()
		self.path_pub = path_pub
		self.pose_pub = pose_pub
		self.head_pub = head_pub
		self.empty_joint_states = JointState()
		self.empty_joint_states.header.stamp = rospy.Time.now()
		self.empty_joint_states.name = ['link_0_to_link_1', 'link_1_to_link_2', 'link_2_to_link_3']
		

	def interpolation_allowed(self, req):
		for joint, constraint in zip([req.joint_1, req.joint_2, req.joint_3], self.constraints):
			if not constraint['min'] <= joint <= constraint['max']:
				rospy.logerr("Joint position out of bounds")
				return False
		
		if req.execution_time <= 0:
			rospy.logerr("Execution time less than 0")
			return False

		if req.interpolation_type != "quadratic" and req.interpolation_type != "linear":
			rospy.logerr("Wrong interpolation type. (available types: linear, quadratic)")
			return False
		return True

	def interpolation_callback(self, req):
		# check for constraints bounds, time and interpolation type (only linear or quadratic)
		if not self.interpolation_allowed(req):
			return False
		
		start_pos = rospy.wait_for_message('joint_states', JointState, timeout = 10).position
		request_pos = [req.joint_1, req.joint_2, req.joint_3]

		rate = rospy.Rate(self.frequency)
		frames_number = int(math.ceil(req.execution_time * self.frequency))
		current_time = 0.

		joint_state = self.empty_joint_states
		if req.interpolation_type == "linear":
			for k in range(0, frames_number + 1):
				positions = []
				for i in range(0, 3):
					positions.append(self.linear_interpolation(start_pos[i], request_pos[i], req.execution_time, current_time))

				joint_state.position = positions
				self.pose_pub.publish(joint_state)
				self.publish_path(joint_state)
				current_time = current_time + 1. / self.frequency
				rate.sleep()
		else:
			coeffs = self.quadratic_coeffs(start_pos, request_pos, req.execution_time) # wspolczynnki funkcji kwadratowych przy drugiej potedze
			for k in range(0, frames_number + 1):
				positions = []
				for i in range(0, 3):
					positions.append(self.quadratic_interpolation(start_pos[i], request_pos[i], req.execution_time, current_time, coeffs[i]))

				joint_state.position = positions
				self.pose_pub.publish(joint_state)
				self.publish_path(joint_state)
				current_time = current_time + 1. / self.frequency
				rate.sleep()

		return True

	def publish_path(self, joint_states):
		head_pose = self.compute_effector_position(joint_states)
		self.path.header = head_pose.header	# fresh header, created in KdlDkin module
		self.path.poses.append(head_pose)
		self.head_pub.publish(head_pose)
		self.path_pub.publish(self.path)

if __name__ == '__main__':
	rospy.init_node('jint_node', anonymous = False)
	pose_pub = rospy.Publisher('interpolation', JointState, queue_size=1)
	path_pub = rospy.Publisher('jint_path', Path, queue_size=1)
	head_pub = rospy.Publisher('head_pose', PoseStamped, queue_size=1)
	rospy.sleep(1)
	params = rospy.get_param('robot_params')

	interpolator = JintInterpolator(10, params, path_pub,  pose_pub, head_pub)
#	rospy.Subscriber("joint_states", JointState, inte.joint_state_callback)
	s = rospy.Service('jint_control_srv', JintControl, interpolator.interpolation_callback)
	rospy.loginfo("Interpolator ready")
	rospy.spin()