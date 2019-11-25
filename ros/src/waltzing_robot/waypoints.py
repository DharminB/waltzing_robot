#! /usr/bin/env python

from __future__ import print_function

import math
import tf
import rospy
from geometry_msgs.msg import PoseArray, Pose
from waltzing_robot.utils import Utils

class Waypoint(object):

    """Class representing a single waypoint"""

    def __init__(self, waypoint_dict=None, default_vel_curve='trapezoid'):
        if waypoint_dict is not None:
            self.x = waypoint_dict.get('x', None)
            self.y = waypoint_dict.get('y', None)
            self.theta = waypoint_dict.get('theta', None)
            self.time = waypoint_dict.get('time', None)
            self.control_points = waypoint_dict.get('control_points', None)
            self.vel_curve = waypoint_dict.get('vel_curve', default_vel_curve)
        else:
            self.x, self.y, self.theta, self.control_points = None, None, None, None
            self.time, self.vel_curve = None, default_vel_curve
        
    def __str__(self):
        string = ''
        string += 'x: ' + str(round(self.x, 3)) + '\n'
        string += 'y: ' + str(round(self.y, 3)) + '\n'
        string += 'theta: ' + str(round(self.theta, 3)) + '\n'
        string += 'time: ' + str(self.time) + '\n'
        string += 'vel_curve: ' + str(self.vel_curve) + '\n'
        string += 'control_points: ' + str(self.control_points) + '\n'
        return string

    def to_pose(self):
        """Return a Pose object representing waypoint
        :returns: geometry_msgs.Pose

        """
        return Utils.get_pose_from_x_y_theta(self.x, self.y, self.theta)

    def shift(self, start_x, start_y, start_theta):
        """Shift the waypoint with the given offsets

        :start_x: float
        :start_y: float
        :start_theta: float
        :returns None

        """
        x, y, theta = self.x, self.y, self.theta
        self.x = start_x + (x * math.cos(start_theta) - y * math.sin(start_theta))
        self.y = start_y + (x * math.sin(start_theta) + y * math.cos(start_theta))
        self.theta += start_theta
        if self.control_points is not None:
            for cp in self.control_points:
                x, y = cp['x'], cp['y']
                cp['x'] = start_x + (x * math.cos(start_theta) - y * math.sin(start_theta))
                cp['y'] = start_y + (x * math.sin(start_theta) + y * math.cos(start_theta))


class Waypoints(object):
    """
    Class representing waypoints used for representing choreography of the 
    dance for the robot
    
    :keyword arguments:
        :waypoint_config: dict{'default_vel_curve': string, 'waypoints': list}
        :waypoints: list of dict of following format
                    {'x': float,
                     'y': float,
                     'theta': float,
                     'time':float,
                     'vel_curve':string,
                     'control_points':list of dict {'x':float, 'y':float}
                    }
    """

    def __init__(self, **kwargs):
        if 'waypoint_config' in kwargs:
            waypoint_config = kwargs.get('waypoint_config')
            self.default_vel_curve = waypoint_config.get('default_vel_curve', 'linear')
            waypoints = waypoint_config.get('waypoints', [])
        else:
            self.default_vel_curve = kwargs.get('default_vel_curve', 'trapezoid')
            waypoints = kwargs.get('waypoints', [])
        self.waypoints = [Waypoint(wp_dict, self.default_vel_curve) for wp_dict in waypoints]

    def __str__(self):
        string = ''
        string += 'default_vel_curve: ' + str(self.default_vel_curve) + '\n'
        string += 'waypoints:' + '\n'
        for wp in self.waypoints:
            string += '  - ' + str(wp).replace('\n', '\n    ')[:-4]
        return string

    def to_pose_array(self, frame):
        """Return a PoseArray object representing waypoints

        :frame: string
        :returns: geometry_msgs.PoseArray

        """
        pose_array = PoseArray()
        pose_array.header.frame_id = frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = [wp.to_pose() for wp in self.waypoints]
        return pose_array

