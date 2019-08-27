#! /usr/bin/env python

from __future__ import print_function
import math
from waltzing_robot.utils import Utils
from waltzing_robot.waypoints import Waypoint, Waypoints

class VelCurveHandler(object):

    """Handle the motion of the robot based on the velocity curve
    
    :max_vel: float
    :max_acc: float
    
    """

    def __init__(self, **kwargs):
        self.max_vel = kwargs.get('max_vel', float('inf'))
        self.max_acc = kwargs.get('max_acc', float('inf'))
        self.trajectory_data_list = list()
        self.trajectory_index = 0

    def __str__(self):
        string = ""
        string += 'max_vel: ' + str(self.max_vel) + '\n'
        string += 'max_acc: ' + str(self.max_acc) + '\n'
        string += 'trajectory_data_list: ' + str(self.trajectory_data_list)
        return string

    def set_params(self, waypoints):
        """Calculates the neccessary values needed for all trajectories defined 
        by set of waypoints. Returns if the list of trajectory is possible or not.

        :waypoints: list of Waypoint objects
        :returns: bool

        """
        # sanity check
        for wp in waypoints:
            assert isinstance(wp, Waypoint)

        # calculate trajectory data
        self.trajectory_data_list = list()
        for i in range(len(waypoints)-1):
            start_wp = waypoints[i]
            end_wp = waypoints[i+1]
            vel_curve = end_wp.vel_curve
            if not (hasattr(self, vel_curve+"_vel") and \
                    callable(getattr(self, vel_curve+"_vel")) and \
                    hasattr(self, vel_curve+"_calc") and \
                    callable(getattr(self, vel_curve+"_calc"))):
                print("Invalid vel curve")
                return False
            curve_specific_data = getattr(self, vel_curve+"_calc")(start_wp, end_wp)
            if curve_specific_data is None: # impossible trajectory
                return False
            self.trajectory_data_list.append(curve_specific_data)
        return True

    def get_vel(self, time_duration, **kwargs):
        if len(self.trajectory_data_list) > self.trajectory_index:
            vel_curve = self.trajectory_data_list[self.trajectory_index]['vel_curve']
            return getattr(self, vel_curve+"_vel")(time_duration, **kwargs)
        else:
            return self.default_vel(time_duration, **kwargs)

    def default_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        return (0.0, 0.0, 0.0)

    def linear_calc(self, start_wp, end_wp):
        data = dict()
        data['vel_curve'] = end_wp.vel_curve
        data['theta'] = Utils.get_shortest_angle(end_wp.theta, start_wp.theta)
        data['time'] = end_wp.time - start_wp.time
        if end_wp.control_points is None:
            data['x'] = end_wp.x - start_wp.x
            data['y'] = end_wp.y - start_wp.y
            data['vel'] = Utils.get_distance(data['x'], data['y'])/data['time']
        else:
            points = [(cp['x'], cp['y']) for cp in end_wp.control_points]
            points.insert(0, (start_wp.x, start_wp.y))
            points.append((end_wp.x, end_wp.y))
            curve_points = Utils.get_spline_curve(points)
            data['curve_points'] = curve_points
            dist = 0
            for i in range(len(curve_points)-1):
                dist += Utils.get_distance_between_points(curve_points[i], curve_points[i+1])
            data['vel'] = dist / data['time']
        return data

    def linear_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        data = self.trajectory_data_list[self.trajectory_index]
        if time_duration >= data['time']:
            return self.default_vel(time_duration, current_position)
        if 'curve_points' in data:
            n = len(data['curve_points'])
            time_offset = data['time'] / (n-1)
            curve_point_index = int(math.floor(time_duration / time_offset))
            x_diff = data['curve_points'][curve_point_index+1][0] - data['curve_points'][curve_point_index][0]
            y_diff = data['curve_points'][curve_point_index+1][1] - data['curve_points'][curve_point_index][1]
            omega = Utils.get_shortest_angle(math.atan2(y_diff, x_diff),
                                             current_position[2])
        else:
            omega = Utils.get_shortest_angle(math.atan2(data['y'], data['x']),
                                             current_position[2])
        return (data['vel'] * math.cos(omega),
                data['vel'] * math.sin(omega),
                data['theta']/data['time'])

    def trapezoid_calc(self, start_wp, end_wp):
        data = dict()
        data['vel_curve'] = end_wp.vel_curve
        data['theta'] = Utils.get_shortest_angle(end_wp.theta, start_wp.theta)
        data['time'] = end_wp.time - start_wp.time
        dist = 0
        if end_wp.control_points is None:
            data['x'] = end_wp.x - start_wp.x
            data['y'] = end_wp.y - start_wp.y
            dist = Utils.get_distance(data['x'], data['y'])
        else:
            points = [(cp['x'], cp['y']) for cp in end_wp.control_points]
            points.insert(0, (start_wp.x, start_wp.y))
            points.append((end_wp.x, end_wp.y))
            curve_points = Utils.get_spline_curve(points)
            data['curve_points'] = curve_points
            for i in range(len(curve_points)-1):
                dist += Utils.get_distance_between_points(curve_points[i], curve_points[i+1])
        discriminant = (data['time'] * self.max_acc)**2 - (4 * self.max_acc * dist)
        if discriminant < 0:
            print("No velocity solution found")
            return None
        des_vel_sol_1 = ((data['time'] * self.max_acc) + (discriminant)**0.5)/2.0
        des_vel_sol_2 = ((data['time'] * self.max_acc) - (discriminant)**0.5)/2.0
        if des_vel_sol_1 <= self.max_vel:
            data['vel'] = des_vel_sol_1
        elif des_vel_sol_2 <= self.max_vel:
            data['vel'] = des_vel_sol_2
        else:
            print("No valid velocity solution found. Found solutions: ", des_vel_sol_1, "and ", des_vel_sol_2)
            return None
        data['acc_time'] = data['vel']/self.max_acc
        data['const_vel_time'] = data['time'] - (2 * data['acc_time'])
        return data

    def trapezoid_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        data = self.trajectory_data_list[self.trajectory_index]
        if time_duration >= data['time']:
            return self.default_vel(time_duration, current_position)
        if 'curve_points' in data:
            n = len(data['curve_points'])
            time_offset = data['time'] / (n-1)
            curve_point_index = int(math.floor(time_duration / time_offset))
            x_diff = data['curve_points'][curve_point_index+1][0] - data['curve_points'][curve_point_index][0]
            y_diff = data['curve_points'][curve_point_index+1][1] - data['curve_points'][curve_point_index][1]
            omega = Utils.get_shortest_angle(math.atan2(y_diff, x_diff),
                                             current_position[2])
        else:
            omega = Utils.get_shortest_angle(math.atan2(data['y'], data['x']),
                                         current_position[2])
        vel = data['vel'] # desired vel
        if time_duration < data['acc_time']: # accelerate
            vel = self.max_acc * time_duration
        elif time_duration > data['time'] - data['acc_time']: # decelerate
            vel -= self.max_acc * (time_duration - data['time'] + data['acc_time'])
        return (vel * math.cos(omega),
                vel * math.sin(omega),
                data['theta']/data['time'])
