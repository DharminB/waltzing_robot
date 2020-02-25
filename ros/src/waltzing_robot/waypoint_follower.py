#! /usr/bin/env python

from __future__ import print_function

import rospy
import math
import copy
import yaml

from geometry_msgs.msg import Twist, PoseArray, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry

from waltzing_robot.waypoints import Waypoints, Waypoint
from waltzing_robot.vel_curve_handler import VelCurveHandler
# from waltzing_robot.music_player import MusicPlayer
from waltzing_robot.utils import Utils

class WaypointFollower(object):

    """Follow waypoints by controlling a holonomic planar mobile robot"""

    def __init__(self):
        # read ros param
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        odom_topic = rospy.get_param('~odom_topic', '/odom')
        localisation_topic = rospy.get_param('~localisation_topic', '/odom')
        self.sleep_duration = rospy.get_param('~sleep_duration', 0.1)
        self.frame = rospy.get_param('~frame', 'odom')
        music_file_name = rospy.get_param('~music_file_name', None)
        # self.music_player = MusicPlayer(music_file_name)

        # class variables
        self._vel_curve_handler = VelCurveHandler(
                max_vel=rospy.get_param('~max_vel', 8.0),
                max_acc=rospy.get_param('~max_acc', 8.0),
                max_dec=rospy.get_param('~max_dec', 8.0),
                allow_unsafe_transition=rospy.get_param('~allow_unsafe_transition', True))
        self.current_position = (0.0, 0.0, 0.0)

        # publishers
        self._cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self._trajectory_pub = rospy.Publisher('~trajectory', PoseArray, queue_size=1)
        self._waypoints_marker_pub = rospy.Publisher('~waypoints_marker', MarkerArray, queue_size=1)

        # subscribers
        self._odom_sub = rospy.Subscriber(odom_topic, Odometry, self._odom_cb)
        # self._localisation_sub = rospy.Subscriber(localisation_topic, PoseWithCovarianceStamped, self._odom_cb)
        
        rospy.sleep(1) # sleep to initialise publishers completely

    def _odom_cb(self, msg):
        self.current_position = Utils.get_x_y_theta_from_pose(msg.pose.pose)

    def _load_waypoint_config_file(self, waypoint_config_filename=None):
        """Load waypoint config file and return Waypoints obj

        :waypoint_config_filename: str
        :returns: Waypoints obj

        """
        try:
            waypoints_dict = None
            with open(waypoint_config_filename, 'r') as file_obj:
                waypoints_dict = yaml.safe_load(file_obj)
            waypoints_obj = Waypoints(waypoint_config=waypoints_dict)
            return waypoints_obj
        except Exception as e:
            rospy.logerr("File could not be read!")
            return None

    def follow_waypoints(self, waypoint_config_filename=None, visualise_trajectory=True):
        """Follow waypoints defined by class variable with their vel curve motion
        Returns true if the whole trajectory was executed completely

        :returns: bool

        """
        # read waypoints from file
        waypoints_obj = self._load_waypoint_config_file(waypoint_config_filename)
        if waypoints_obj is None:
            return False

        # convert wp to local frame
        start_pose = copy.deepcopy(self.current_position)
        x, y, theta = start_pose
        for wp in waypoints_obj.waypoints:
            wp.shift(*start_pose)
        self._waypoints_marker_pub.publish(
                waypoints_obj.to_marker_array(self.frame, x, y, theta))

        # create dummy wp out of current position
        current = Waypoint({'x': x, 'y': y, 'theta': theta, 'time':0.0})
        waypoints = waypoints_obj.waypoints
        waypoints.insert(0, current)

        # check if waypoint trajectory is possible
        possible = self._vel_curve_handler.set_params(waypoints)
        if not possible:
            rospy.logwarn("Impossible trajectory encountered. Giving up.")
            return False

        if visualise_trajectory:
            self.visualise_trajectory(waypoints)
        self._vel_curve_handler.reset_trajectory_data()
        # self.music_player.start_playing()
        start_time = rospy.get_time()
        last_wp_time = 0.0

        # iterate over all wp and execute trajectory
        for self._vel_curve_handler.trajectory_index, wp in enumerate(waypoints[1:]):
            print(wp)
            while start_time + wp.time > rospy.get_time():
                if rospy.is_shutdown():
                    self.publish_zero_vel()
                    return False

                x, y, theta = self._vel_curve_handler.get_vel(
                        rospy.get_time() - start_time - last_wp_time,
                        current_position=self.current_position)
                # print(x, y, theta)
                self._cmd_vel_pub.publish(self._get_twist(x, y, theta))

                rospy.sleep(self.sleep_duration)
            last_wp_time = wp.time
        end_time = rospy.get_time()
        print("Trajectory executed in", end_time - start_time, "seconds")
        self.publish_zero_vel()
        # self.music_player.stop_playing()
        return True

    def visualise_trajectory(self, waypoints):
        """Publish PoseArray representing the trajectory
        :waypoints: list of Waypoint obj
        :returns: None

        """
        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame
        pose_array.header.stamp = rospy.Time.now()
        poses = []
        current_time = 0.0
        delta_time = 0.1
        x, y, theta = self.current_position
        last_wp_time = 0.0
        for self._vel_curve_handler.trajectory_index, wp in enumerate(waypoints[1:]):
            while current_time < wp.time:
                # current position has to be given (0,0,0) because the cmd_vel
                # are applied in robot's frame whereas the poses are published
                # in global frame
                x_vel, y_vel, theta_vel = self._vel_curve_handler.get_vel(
                        current_time - last_wp_time,
                        current_position=(x, y, theta))
                        # current_position=(0.0, 0.0, 0.0))
                x += x_vel*math.cos(theta)*delta_time - y_vel*math.sin(theta)*delta_time
                y += x_vel*math.sin(theta)*delta_time + y_vel*math.cos(theta)*delta_time
                theta += theta_vel*delta_time
                pose = Utils.get_pose_from_x_y_theta(x, y, theta)
                poses.append(pose)
                current_time += delta_time
                # time.sleep(delta_time)
                # pose_array.poses = poses
                # self._trajectory_pub.publish(pose_array)
            last_wp_time = wp.time
        pose_array.poses = poses
        self._trajectory_pub.publish(pose_array)

    def _get_twist(self, x=0.0, y=0.0, theta=0.0):
        """Return twist ros message object.

        :x: float
        :y: float
        :theta: float
        :returns: geometry_msgs.msg.Twist

        """
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = theta
        return msg

    def publish_zero_vel(self):
        self._cmd_vel_pub.publish(self._get_twist())

