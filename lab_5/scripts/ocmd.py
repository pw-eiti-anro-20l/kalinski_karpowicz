#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from lab_4.srv import OintControl
from math import atan2, sqrt, pi, sin, cos
import numpy as np
from sympy import Point as SympyPoint
from sympy import Ellipse
from lab_5.srv import VisualizationControl



class Point:
    def __init__(self, x, y, z):
        self.x=x
        self.y=y
        self.z=z

class ShapeDrawer(object):
    def __init__(self, interpolation_type):
        self.interpolation_type = interpolation_type
        head_pose = rospy.wait_for_message('/head_pose', PoseStamped, timeout=3)
        p = head_pose.pose.position
        self.actual_pos = Point(p.x, p.y, p.z)
        self.plane_y = 0.1 #we will be drawing on y plane 0.1m from original y_plane
        self.x_range = {'min':-0.4, 'max':0.4}
        self.z_range = {'min':-0.4, 'max':0.4}
        self.service_caller = rospy.ServiceProxy('/oint_control_srv', OintControl)
        self.interval = 2
        self.choice = {'rectangle':self.draw_rectangle, '3d_rectangle':self.draw_3d_rectangle, \
            'ellipse':self.draw_ellipse, '3d_ellipse':self.draw_3d_ellipse}

    def draw_3d_rectangle(self, n_cycles):
        rect_path = [Point(self.x_range['max'], self.plane_y, self.z_range['min']), \
            Point(self.x_range['max'], self.plane_y, self.z_range['max']), \
            Point(self.x_range['min'], self.plane_y, self.z_range['max']), \
            Point(self.x_range['min'], self.plane_y, self.z_range['min'])]
        rect_path.append(rect_path[0])     
        for i in range(n_cycles):
            for p in rect_path:
                try:
                    self.service_caller(x=p.x, y=p.y+i*0.1, z=p.z, qx=0, qy=0, qz=0, qw=1, \
                        execution_time=self.interval, interpolation_type=self.interpolation_type)
                except rospy.ServiceException as exc:
                    pass
    
    def draw_rectangle(self, n_cycles):
        rect_path = [Point(self.x_range['max'], self.plane_y, self.z_range['min']), \
            Point(self.x_range['max'], self.plane_y, self.z_range['max']), \
            Point(self.x_range['min'], self.plane_y, self.z_range['max']), \
            Point(self.x_range['min'], self.plane_y, self.z_range['min'])]
        rect_path.append(rect_path[0])
        for n in range(n_cycles):
            for p in rect_path:
                try:
                    self.service_caller(x=p.x, y=p.y, z=p.z, qx=0, qy=0, qz=0, qw=1, \
                        execution_time=self.interval, interpolation_type=self.interpolation_type)
                except rospy.ServiceException as exc:
                    pass

    def draw_ellipse(self, n_cycles):
        a = 1
        b = 2
        x = self.x_range['max']
        z = self.x_range['min']
        ls = np.linspace(0, 2*np.pi, num=50)
        for _ in range(n_cycles):
            for i in ls:
                e1 = Ellipse(SympyPoint(0, 0), a, b)
                p = e1.arbitrary_point()
              #  print('x='+str(cos(i)) + '  y= '+str(2*sin(i)))
                x=cos(i)/2
                z=3*sin(i)/10
                try:
                    self.service_caller(x=x, y=0.1, z=z, qx=0, qy=0, qz=0, qw=1, \
                        execution_time=0.01, interpolation_type=self.interpolation_type)
                except rospy.ServiceException as exc:
                    pass
        return True

    def draw_3d_ellipse(self, n_cycles):
        a = 1
        b = 2
        x = self.x_range['max']
        z = self.x_range['min']
        ls = np.linspace(0, 2*np.pi, num=50)
        for n in range(n_cycles):
            for i in ls:
                e1 = Ellipse(SympyPoint(0, 0), a, b)
                p = e1.arbitrary_point()
              #  print('x='+str(cos(i)) + '  y= '+str(2*sin(i)))
                x=cos(i)/2
                z=3*sin(i)/10
                try:
                    self.service_caller(x=x, y=0.1+n*0.1, z=z, qx=0, qy=0, qz=0, qw=1, \
                        execution_time=0.01, interpolation_type=self.interpolation_type)
                except rospy.ServiceException as exc:
                    pass
        return True
    def visualization_callback(self, req):
        try:
            s = self.choice[str(req.shape)]
            print(s(req.n_cycles))
        except KeyError:
            rospy.logwarn('incorrect shape type. Available types: rectangle, 3d_rectangle, ellipse, 3d_ellipse')
            return False
        rospy.loginfo('visualization done')
        return True

if __name__ == "__main__":
    rospy.init_node('shape_drawer_node', anonymous = False)
    sd = ShapeDrawer('linear')
    service = rospy.Service('VisualizationControl', VisualizationControl, sd.visualization_callback)
    rospy.loginfo('waiting for visualizations requests')
    rospy.spin()