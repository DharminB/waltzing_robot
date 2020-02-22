#! /usr/bin/env python

from __future__ import print_function
import math
import copy
import traceback
from waltzing_robot.utils import Utils
from waltzing_robot.waypoints import Waypoint, Waypoints
from waltzing_robot.trapezoid_velocity_calculator import TrapezoidVelocityCalculator

class VelCurveHandler(object):

    """Handle the motion of the robot based on the velocity curve
    
    :max_vel: float
    :max_acc: float
    :max_dec: float
    
    """

    def __init__(self, **kwargs):
        self.max_vel = kwargs.get('max_vel', float('inf'))
        self.max_acc = kwargs.get('max_acc', float('inf'))
        self.max_dec = kwargs.get('max_dec', float('inf'))
        self.curve_point_distance_tolerance = 0.05
        self.trajectory_data_list = list()
        self.trajectory_index = 0
        self.traj_vel_calc = TrapezoidVelocityCalculator(max_acc=self.max_acc,
                                                         max_dec=self.max_dec,
                                                         max_vel=self.max_vel)

        self.allow_unsafe_transition = kwargs.get('allow_unsafe_transition', True)
        tos_radius = kwargs.get('turn_on_spot_radius', 0.1) # turn on spot radius
        self.tos_time = kwargs.get('turn_on_spot_time', 1.5)
        self.turn_on_spot_wp = Waypoint(
            waypoint_dict={
                'x': 0.0, 'y':0.0, 'theta':0.0, 'time': 1.0,
                'control_points': [{'x': tos_radius, 'y': 0.0}, {'x': 2*tos_radius, 'y': -2*tos_radius},
                                   {'x': 2*tos_radius, 'y': 2*tos_radius}, {'x': tos_radius, 'y': 0.0}]},
            default_vel_curve='trapezoid')

    def __str__(self):
        string = ""
        string += 'max_vel: ' + str(self.max_vel) + '\n'
        string += 'max_acc: ' + str(self.max_acc) + '\n'
        string += 'max_dec: ' + str(self.max_dec) + '\n'
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
        i = 0
        # for i in range(len(waypoints)-1):
        while i < len(waypoints) - 1:
            start_wp = waypoints[i]
            end_wp = waypoints[i+1]
            # print(start_wp, end_wp)
            # print(i)
            if i < len(waypoints) - 2:
                safe = self._is_transition_safe(waypoints[i], waypoints[i+1], waypoints[i+2])
                if not safe:
                    print("NOT SAFE")
                    intermediate_wp = self._get_safe_intermediate_wp(waypoints[i], waypoints[i+1])
                    print(intermediate_wp)
                    waypoints.insert(i+2, intermediate_wp)
                    # print(waypoints)
                    # print("\n"*3)
            vel_curve = end_wp.vel_curve
            if not (hasattr(self, vel_curve+"_vel") and \
                    callable(getattr(self, vel_curve+"_vel")) and \
                    hasattr(self, vel_curve+"_calc") and \
                    callable(getattr(self, vel_curve+"_calc"))):
                print("Invalid vel curve")
                return False
            try:
                curve_specific_data = getattr(self, vel_curve+"_calc")(start_wp, end_wp)
            except Exception as e:
                print('ERROR: ' + str(e))
                traceback.print_exc()
                curve_specific_data = None
            if curve_specific_data is None: # impossible trajectory
                return False
            self.trajectory_data_list.append(curve_specific_data)
            i += 1
        return True

    def _is_transition_safe(self, start_wp, middle_wp, end_wp):
        """Check if the direction of travel changes are within threshold

        :start_wp: Waypoint
        :middle_wp: Waypoint
        :end_wp: Waypoint
        :returns: bool

        """
        if self.allow_unsafe_transition:
            return True
        # print("inside is_transition_safe")
        if middle_wp.control_points is None:
            dir_bw_start_middle = math.atan2(middle_wp.y - start_wp.y, middle_wp.x - start_wp.x)
        else:
            dir_bw_start_middle = math.atan2(middle_wp.y - middle_wp.control_points[-1]['y'],
                                             middle_wp.x - middle_wp.control_points[-1]['x'])
        if end_wp.control_points is None:
            dir_bw_middle_end = math.atan2(end_wp.y - middle_wp.y, end_wp.x - middle_wp.x)
        else:
            dir_bw_middle_end = math.atan2(end_wp.control_points[-1]['y'] - middle_wp.y,
                                           end_wp.control_points[-1]['x'] - middle_wp.x)
        if abs(dir_bw_start_middle - dir_bw_middle_end) < 1.6:
            return True
        return False

    def _get_safe_intermediate_wp(self, start_wp, middle_wp):
        """Return a waypoint which makes the transition safe

        :start_wp: Waypoint
        :middle_wp: Waypoint
        :returns: Waypoint

        """
        if middle_wp.control_points is None:
            dir_bw_start_middle = math.atan2(middle_wp.y - start_wp.y, middle_wp.x - start_wp.x)
        else:
            dir_bw_start_middle = math.atan2(middle_wp.y - middle_wp.control_points[-1]['y'],
                                             middle_wp.x - middle_wp.control_points[-1]['x'])
        intermediate_wp = copy.deepcopy(self.turn_on_spot_wp)
        intermediate_wp.shift(middle_wp.x, middle_wp.y, dir_bw_start_middle)
        intermediate_wp.time = middle_wp.time
        intermediate_wp.theta = middle_wp.theta
        middle_wp.time -= self.tos_time
        return intermediate_wp

    def get_vel(self, time_duration, **kwargs):
        if len(self.trajectory_data_list) > self.trajectory_index:
            vel_curve = self.trajectory_data_list[self.trajectory_index]['vel_curve']
            return getattr(self, vel_curve+"_vel")(time_duration, **kwargs)
        else:
            return self.default_vel(time_duration, **kwargs)

    def reset_trajectory_data(self):
        for trajectory_data in self.trajectory_data_list:
            if 'curve_point_index' in trajectory_data:
                trajectory_data['curve_point_index'] = 1

    def default_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        return (0.0, 0.0, 0.0)

    def linear_calc(self, start_wp, end_wp):
        data = dict()
        data['vel_curve'] = end_wp.vel_curve
        data['theta'] = Utils.get_shortest_angle(end_wp.theta, start_wp.theta)
        data['time'] = end_wp.time - start_wp.time
        if end_wp.control_points is None:
            data['delta_x'] = end_wp.x - start_wp.x
            data['delta_y'] = end_wp.y - start_wp.y
            data['end_wp'] = (end_wp.x, end_wp.y, end_wp.theta)
            data['vel'] = Utils.get_distance(data['delta_x'], data['delta_y'])/data['time']
        else:
            points = [(cp['x'], cp['y']) for cp in end_wp.control_points]
            points.insert(0, (start_wp.x, start_wp.y))
            points.append((end_wp.x, end_wp.y))
            n = Utils.calc_heuristic_n_from_points(points)
            curve_points = Utils.get_spline_curve(points, n=n)
            data['curve_points'] = curve_points
            distances = [0.0]
            for i in range(len(curve_points)-1):
                distances.append(distances[-1] +\
                                 Utils.get_distance_between_points(curve_points[i],
                                                                   curve_points[i+1]))
            data['distances'] = distances
            dist = distances[-1]
            data['vel'] = dist / data['time']
            data['curve_point_index'] = 1
        return data

    def linear_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        data = self.trajectory_data_list[self.trajectory_index]
        if time_duration >= data['time']:
            return self.default_vel(time_duration, current_position)
        if 'curve_points' in data: # spline curve
            dist_to_cp = Utils.get_distance_between_points(
                    current_position[:2],
                    data['curve_points'][data['curve_point_index']])
            total_remaining_distance = dist_to_cp + data['distances'][-1] -\
                                       data['distances'][data['curve_point_index']]
            if dist_to_cp < self.curve_point_distance_tolerance:
                data['curve_point_index'] += 1
                if data['curve_point_index'] == len(data['curve_points']):
                    data['curve_point_index'] = -1
                    return self.default_vel(time_duration, current_position)

            curve_point_index = data['curve_point_index']
            x_diff = data['curve_points'][curve_point_index][0] - current_position[0]
            y_diff = data['curve_points'][curve_point_index][1] - current_position[1]
            omega = Utils.get_shortest_angle(math.atan2(y_diff, x_diff),
                                             current_position[2])
        else: # straight motion
            omega = Utils.get_shortest_angle(math.atan2(data['delta_y'], data['delta_x']),
                                             current_position[2])
            total_remaining_distance = Utils.get_distance_between_points(
                    current_position[:2],
                    data['end_wp'][:2])
        total_remaining_time = data['time'] - time_duration
        req_vel = total_remaining_distance / total_remaining_time
        # print(data['vel'], req_vel)
        vel = data['vel']
        if abs(req_vel - data['vel']) <= self.max_acc:
            vel = req_vel
        else:
            sign = 1 if req_vel > data['vel'] else -1
            vel = data['vel'] + (sign * self.max_acc)
        return (vel * math.cos(omega),
                vel * math.sin(omega),
                data['theta']/data['time'])

    def trapezoid_calc(self, start_wp, end_wp):
        data = dict()
        data['vel_curve'] = end_wp.vel_curve
        data['theta'] = Utils.get_shortest_angle(end_wp.theta, start_wp.theta)
        data['time'] = end_wp.time - start_wp.time
        dist = 0
        if end_wp.control_points is None:
            data['delta_x'] = end_wp.x - start_wp.x
            data['delta_y'] = end_wp.y - start_wp.y
            data['end_wp'] = (end_wp.x, end_wp.y, end_wp.theta)
            # data['vel'] = Utils.get_distance(data['delta_x'], data['delta_y'])/data['time']
            dist = Utils.get_distance(data['delta_x'], data['delta_y'])
        else:
            points = [(cp['x'], cp['y']) for cp in end_wp.control_points]
            points.insert(0, (start_wp.x, start_wp.y))
            points.append((end_wp.x, end_wp.y))
            n = Utils.calc_heuristic_n_from_points(points)
            curve_points = Utils.get_spline_curve(points, n=n)
            data['curve_points'] = curve_points
            distances = [0.0]
            for i in range(len(curve_points)-1):
                distances.append(distances[-1] +\
                                 Utils.get_distance_between_points(curve_points[i],
                                                                   curve_points[i+1]))
            data['distances'] = distances
            dist = distances[-1]
            data['curve_point_index'] = 1

        desired_vel_list = self.traj_vel_calc.calc_desired_vel(distance=dist, time=data['time'])
        if len(desired_vel_list) == 0:
            print("No velocity solution found")
            return None

        desired_vel = desired_vel_list[0]
        data['vel'] = desired_vel['vel']
        data['acc_time'] = desired_vel['t_acc']
        data['const_vel_time'] = desired_vel['t_const_vel']
        data['dec_time'] = desired_vel['t_dec']
        return data

    def trapezoid_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        data = self.trajectory_data_list[self.trajectory_index]
        # print()
        # print(self.trajectory_index)
        # print(current_position)
        # print(data['curve_points'][data['curve_point_index']])
        # print(data)
        if time_duration >= data['time']:
            return self.default_vel(time_duration, current_position)
        if 'curve_points' in data:
            dist_to_cp = Utils.get_distance_between_points(
                    current_position[:2],
                    data['curve_points'][data['curve_point_index']])
            total_remaining_distance = dist_to_cp + data['distances'][-1] -\
                                       data['distances'][data['curve_point_index']]
            if dist_to_cp < self.curve_point_distance_tolerance and data['curve_point_index'] < len(data['curve_points'])-1:
                data['curve_point_index'] += 1
                # if data['curve_point_index'] == len(data['curve_points']):
                #     data['curve_point_index'] = -1
                    # return self.default_vel(time_duration, current_position)

            curve_point_index = data['curve_point_index']
            # print(curve_point_index)
            x_diff = data['curve_points'][curve_point_index][0] - current_position[0]
            y_diff = data['curve_points'][curve_point_index][1] - current_position[1]
            # print(x_diff, y_diff)
            omega = Utils.get_shortest_angle(math.atan2(y_diff, x_diff),
                                             current_position[2])
        else:
            omega = Utils.get_shortest_angle(math.atan2(data['delta_y'], data['delta_x']),
                                             current_position[2])
            total_remaining_distance = Utils.get_distance_between_points(
                    current_position[:2],
                    data['end_wp'][:2])
            if total_remaining_distance < self.curve_point_distance_tolerance:
                total_remaining_distance = 0.0
        total_remaining_time = data['time'] - time_duration
        req_vel = total_remaining_distance / total_remaining_time
        if total_remaining_time < 0.1: # if no time left, dont bother correcting
            req_vel = data['vel']

        # print('req_vel', req_vel)
        # print('total_remaining_distance', total_remaining_distance)
        # print('total_remaining_time', total_remaining_time)
        vel = data['vel']
        if abs(req_vel - data['vel']) <= self.max_acc:
            vel = req_vel
        else:
            sign = 1 if req_vel > data['vel'] else -1
            vel = data['vel'] + (sign * self.max_acc)
        if time_duration < data['acc_time']: # accelerate
            vel = self.max_acc * time_duration
        elif time_duration > data['time'] - data['dec_time']: # decelerate
            vel -= self.max_acc * (time_duration - data['time'] + data['dec_time'])
        # print('vel', vel)
        # print('omega', omega)
        # print(vel * math.cos(omega), vel * math.sin(omega), data['theta']/data['time'])
        return (vel * math.cos(omega),
                vel * math.sin(omega),
                data['theta']/data['time'])
