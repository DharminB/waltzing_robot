#! /usr/bin/env python

from __future__ import print_function
import math
from waltzing_robot.utils import Utils

class VelCurveHandler(object):

    """Handle the motion of the robot based on the velocity curve
    
    :max_vel: float
    :max_acc: float
    
    """

    def __init__(self, **kwargs):
        self.max_vel = kwargs.get('max_vel', float('inf'))
        self.max_acc = kwargs.get('max_acc', float('inf'))
        self.x, self.y, self.theta = None, None, None
        self.time, self.vel_curve = None, "default"
        self.curve_specific_data = dict()

    def __str__(self):
        string = ""
        string += 'max_vel: ' + str(self.max_vel) + '\n'
        string += 'max_acc: ' + str(self.max_acc) + '\n'
        string += 'delta_x: ' + str(self.x) + '\n'
        string += 'delta_y: ' + str(self.y) + '\n'
        string += 'delta_theta: ' + str(self.theta) + '\n'
        string += 'delta_time: ' + str(self.time) + '\n'
        string += 'vel_curve: ' + str(self.vel_curve) + '\n'
        string += 'curve_data: ' + str(self.curve_specific_data)
        return string

    def set_delta(self, delta_dict):
        """set the delta dict containing all neccessary info for trajectory generation

        :delta_dict: dict
        :returns: None

        """
        self.x = delta_dict['x']
        self.y = delta_dict['y']
        self.theta = delta_dict['theta']
        self.time = delta_dict['time']
        self.vel_curve = delta_dict['vel_curve']
        if not (hasattr(self, self.vel_curve+"_vel") and callable(getattr(self, self.vel_curve+"_vel"))):
            print("Invalid vel curve. Resetting to default")
            self.vel_curve = "default"
            return False
        if hasattr(self, self.vel_curve+"_calc") and callable(getattr(self, self.vel_curve+"_calc")):
            return getattr(self, self.vel_curve+"_calc")()
        return False

    def get_vel(self, time_duration, **kwargs):
        return getattr(self, self.vel_curve+"_vel")(time_duration, **kwargs)

    def default_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        return (0.0, 0.0, 0.0)

    def linear_calc(self):
        self.curve_specific_data['vel'] = Utils.get_distance(self.x, self.y)/self.time
        return True

    def linear_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        if time_duration >= self.time:
            self.vel_curve = "default"
            return (0.0, 0.0, 0.0)
        omega = Utils.get_shortest_angle(math.atan2(self.y, self.x), current_position[2])
        vel = self.curve_specific_data['vel']
        return (vel * math.cos(omega), vel * math.sin(omega), self.theta/self.time)

    def trapezoid_calc(self):
        dist = Utils.get_distance(self.x, self.y)
        discriminant = (self.time * self.max_acc)**2 - (4 * self.max_acc * dist)
        if discriminant < 0:
            print("No velocity solution found")
            return False
        des_vel_sol_1 = ((self.time * self.max_acc) + (discriminant)**0.5)/2.0
        des_vel_sol_2 = ((self.time * self.max_acc) - (discriminant)**0.5)/2.0
        if des_vel_sol_1 <= self.max_vel:
            self.curve_specific_data['vel'] = des_vel_sol_1
        elif des_vel_sol_2 <= self.max_vel:
            self.curve_specific_data['vel'] = des_vel_sol_2
        else:
            print("No valid velocity solution found. Found solutions: ", des_vel_sol_1, "and ", des_vel_sol_2)
            return False
        self.curve_specific_data['acc_time'] = self.curve_specific_data['vel']/self.max_acc
        self.curve_specific_data['const_vel_time'] = self.time - (2 * self.curve_specific_data['acc_time'])
        print(self.curve_specific_data)
        return True

    def trapezoid_vel(self, time_duration, current_position=(0.0, 0.0, 0.0)):
        if time_duration >= self.time:
            self.vel_curve = "default"
            return (0.0, 0.0, 0.0)
        omega = Utils.get_shortest_angle(math.atan2(self.y, self.x), current_position[2])
        vel = self.curve_specific_data['vel'] # desired vel
        if time_duration < self.curve_specific_data['acc_time']:
            # accelerate
            vel = self.max_acc * time_duration
        elif time_duration > self.time - self.curve_specific_data['acc_time']:
            # decelerate
            vel -= self.max_acc * (time_duration - self.time + self.curve_specific_data['acc_time'])
        return (vel * math.cos(omega), vel * math.sin(omega), self.theta/self.time)
