#! /usr/bin/env python

from __future__ import print_function

import math
import tf
import rospy
import copy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseArray, Pose, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
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

    def to_marker_array(self, frame, init_x=0.0, init_y=0.0, init_theta=0.0):
        """Return a MarkerArray object representing the trajectory formed by waypoints

        :frame: string
        :returns: visualization_msgs.MarkerArray

        """
        waypoints = copy.deepcopy(self)
        waypoints.waypoints.insert(0, Waypoint(waypoint_dict={'x': init_x,
                                                              'y': init_y,
                                                              'theta': init_theta,
                                                              'time': 0.0},
                                               default_vel_curve=self.default_vel_curve))
        marker_array = MarkerArray()
        for i, wp in enumerate(waypoints.waypoints):
            marker = Marker(pose=wp.to_pose(), type=Marker.ARROW, id=i)
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = frame
            marker.scale = Vector3(0.5, 0.1, 0.1)
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            if i == len(waypoints.waypoints)-1:
                marker.color.a = 0.9
            else:
                marker.color.a = 0.5
            marker_array.markers.append(marker)

            marker_msg_text = Marker(type=Marker.TEXT_VIEW_FACING,
                                     id=20000+i,
                                     text="t=" + str(wp.time),
                                     scale=Vector3(0.0, 0.0, 0.15),
                                     color=ColorRGBA(1., 1., 1., 1.))
            marker_msg_text.pose = wp.to_pose()
            # marker_msg_text.pose.position.y += 0.1
            marker_msg_text.header.stamp = rospy.Time.now()
            marker_msg_text.header.frame_id = frame
            marker_array.markers.append(marker_msg_text)

        for i in range(1, len(waypoints.waypoints)):
            start_wp = waypoints.waypoints[i-1]
            end_wp = waypoints.waypoints[i]
            if end_wp.control_points is None:
                marker = Marker(type=Marker.LINE_STRIP, id=10000+i)
                end_wp_pose = waypoints.waypoints[i].to_pose()
                start_wp_pose = waypoints.waypoints[i-1].to_pose()
                marker.points.append(start_wp_pose.position)
                marker.points.append(end_wp_pose.position)
            else:
                marker = Marker(type=Marker.LINE_LIST, id=10000+i)
                points = [(cp['x'], cp['y']) for cp in end_wp.control_points]
                points.insert(0, (start_wp.x, start_wp.y))
                points.append((end_wp.x, end_wp.y))
                n = Utils.calc_heuristic_n_from_points(points)
                curve_points = Utils.get_spline_curve(points, n=n)
                curve_ros_points = [Point(x=p[0], y=p[1], z=0.0) for p in curve_points]
                for j, p in enumerate(curve_ros_points):
                    marker.points.append(p)
                    if j == 0 or j == len(curve_ros_points)-1:
                        continue
                    marker.points.append(copy.deepcopy(p))
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = frame
            marker.color.g = marker.color.a = 1.0
            if i == len(waypoints.waypoints)-1:
                marker.scale.x = 0.02
            else:
                marker.scale.x = 0.01
            marker_array.markers.append(marker)
        return marker_array
