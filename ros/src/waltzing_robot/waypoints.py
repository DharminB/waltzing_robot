#! /usr/bin/env python

from __future__ import print_function

import tf
import rospy
from geometry_msgs.msg import PoseArray, Pose

class Waypoint(object):

    """Class representing a single waypoint"""

    def __init__(self, waypoint_dict=None, default_vel_curve='square'):
        if waypoint_dict is not None:
            self.x = waypoint_dict.get('x', None)
            self.y = waypoint_dict.get('y', None)
            self.theta = waypoint_dict.get('theta', None)
            self.time = waypoint_dict.get('time', None)
            self.vel_curve = waypoint_dict.get('vel_curve', default_vel_curve)
        else:
            self.x, self.y, self.theta = None, None, None
            self.time, self.vel_curve = None, default_vel_curve
        
    def __str__(self):
        string = ''
        string += 'x: ' + str(self.x) + '\n'
        string += 'y: ' + str(self.y) + '\n'
        string += 'theta: ' + str(self.theta) + '\n'
        string += 'time: ' + str(self.time) + '\n'
        string += 'vel_curve: ' + str(self.vel_curve) + '\n'
        return string

    def to_pose(self):
        """Return a Pose object representing waypoint
        :returns: geometry_msgs.Pose

        """
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

class Waypoints(object):
    """
    Class representing waypoints used for representing choreography of the 
    dance for the robot
    
    :keyword arguments:
        :waypoint_config: dict{'frame': string, 'waypoints': list}
        :waypoints: list of dict{'x': float, 'y': float, 'theta': float, 'time':float}
        :frame: string

    """

    def __init__(self, **kwargs):
        if 'waypoint_config' in kwargs:
            waypoint_config = kwargs.get('waypoint_config')
            self.frame = waypoint_config.get('frame', '')
            self.default_vel_curve = waypoint_config.get('default_vel_curve', 'square')
            waypoints = waypoint_config.get('waypoints', [])
        else:
            self.frame = kwargs.get('frame', '')
            self.default_vel_curve = kwargs.get('default_vel_curve', 'square')
            waypoints = kwargs.get('waypoints', [])
        self.waypoints = [Waypoint(wp_dict, self.default_vel_curve) for wp_dict in waypoints]

    def __str__(self):
        string = ''
        string += 'frame: ' + self.frame + '\n'
        string += 'waypoints:' + '\n'
        for wp in self.waypoints:
            string += '  - ' + str(wp).replace('\n', '\n    ')[:-4]
        return string

    def to_pose_array(self):
        """Return a PoseArray object representing waypoints
        :returns: geometry_msgs.PoseArray

        """
        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = [wp.to_pose() for wp in self.waypoints]
        return pose_array

