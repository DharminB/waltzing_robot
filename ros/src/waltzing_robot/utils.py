#! /usr/bin/env python

import tf
import rospy
import math
from geometry_msgs.msg import Pose, TransformStamped, Quaternion
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback

class Utils(object):

    """Utility functions used for waltzing robot code stack"""

    @staticmethod
    def get_pose_from_x_y_theta(x, y, theta):
        """Return a Pose object from x, y and theta
        :x: float
        :y: float
        :theta: float
        :returns: geometry_msgs.Pose

        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        pose.orientation = Quaternion(*quat)
        return pose

    @staticmethod
    def get_x_y_theta_from_pose(pose):
        """Return a tuple(x, y, theta) from Pose objects

        :pose: geometry_msgs/Pose
        :returns: tuple(x, y, theta)

        """
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        return (pose.position.x, pose.position.y, theta)

    @staticmethod
    def get_static_transform_from_x_y_theta(x, y, theta, frame_id="odom"):
        """Create a TransformedStamped message from x y and theta to 0, 0 and 0
        
        :x: float
        :y: float
        :theta: float
        :returns: geometry_msgs.TransformStamped
        """
        transform = TransformStamped()
  
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = frame_id
        transform.child_frame_id = "start_pose"
  
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
  
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        transform.transform.rotation = Quaternion(*quat)
        return transform

    @staticmethod
    def get_shortest_angle(angle1, angle2):
        """Compute the angular distance between two angles (in radians)

        :angle1: float
        :angle2: float
        :returns: float
        """
        return math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2))

    @staticmethod
    def get_distance(delta_x, delta_y):
        """Compute cartesian distance given individual distance in x and y axis

        :delta_x: float
        :delta_y: float
        :returns: float

        """
        return (delta_x**2 + delta_y**2)**0.5

    @staticmethod
    def get_distance_between_points(p1, p2):
        """Compute cartesian distance given two points

        :p1: tuple(float, float)
        :p2: tuple(float, float)
        :returns: float

        """
        return Utils.get_distance(p1[0]-p2[0], p1[1]-p2[1])

    @staticmethod
    def get_spline_curve(points, n=11):
        """Return `n` points on spline curve defined by `points` where first and 
        last point is start and end point and all the points in the middle are 
        control points.

        :points: list of tuple(float, float)
        :n: int
        :returns: list of tuple(float, float)

        """
        order = len(points) - 1
        coef = Utils.pascal_triangle_row(order)
        offset = 1.0 / (n-1)
        curve_points = []
        curve_points.append(points[0]) # add start point as first curve point
        # add n-2 curve points in middle
        for factor in range(1, n-1):
            t = offset * factor
            x, y = 0.0, 0.0
            for i in range(order+1):
                x += coef[i] * (1-t)**(order-i) * t**(i) * points[i][0]
                y += coef[i] * (1-t)**(order-i) * t**(i) * points[i][1]
            curve_points.append((x, y))
        curve_points.append(points[-1]) # add end point as last curve point
        return curve_points

    @staticmethod
    def pascal_triangle_row(n):
        """Returns `n`th row from pascal triangle.

        :n: int
        :returns: list of int

        """
        coef = [1]
        for i in range(1, n+1):
            coef.append(int(coef[-1] * ((n + 1.0 - i)/i)))
        return coef

    @staticmethod
    def calc_heuristic_n_from_points(points):
        dist = 0.0
        for i in range(len(points)-1):
            dist += Utils.get_distance_between_points(points[i], points[i+1])
        return max(int(dist)*2, 10)

    @staticmethod
    def get_2_dof_interactive_marker(marker_name, frame, x=0.0, y=0.0):
        """Return an interactive marker with 2 degree of freedom (X and Y axis)
        in `frame` at (`x`, `y`, 0.0) position named `name`

        :marker_name: string
        :frame: string
        :x: int
        :y: int
        :returns: visualization_msgs.InteractiveMarker

        """
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame
        int_marker.name = marker_name
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        # int_marker.description = "Simple 2-DOF Control"

        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.SPHERE
        box_marker.scale.x = box_marker.scale.y = box_marker.scale.z = 0.1
        box_marker.color.r = box_marker.color.a = 1.0
        box_marker.color.g = box_marker.color.b = 0.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )

        # add the control to the interactive marker
        int_marker.controls.append( box_control )

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        # add the control to the interactive marker
        int_marker.controls.append(rotate_control);

        rotate_control2 = InteractiveMarkerControl()
        rotate_control2.orientation.z = rotate_control2.orientation.w = 0.707
        rotate_control2.name = "move_y"
        rotate_control2.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        # add the control to the interactive marker
        int_marker.controls.append(rotate_control2);
        return int_marker
