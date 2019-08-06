#! /usr/bin/env python

class VelCurveHandler(object):

    """Handle the motion of the robot based on the velocity curve
    
    :max_vel: float
    :max_acc: float
    
    """

    def __init__(self, **kwargs):
        self.max_vel = kwargs.get('max_vel', float('inf'))
        self.max_acc = kwargs.get('max_acc', float('inf'))
        self.x, self.y, self.theta = None, None, None
        self.time, self.vel_curve = None, None
        self._vel_curve_func = {'default': self.default_vel,
                           'square': self.square_vel,
                           'linear': self.linear_vel}
        self.get_vel = self._vel_curve_func['default']

    def __str__(self):
        string = ""
        string += 'max_vel: ' + str(self.max_vel) + '\n'
        string += 'max_acc: ' + str(self.max_acc) + '\n'
        string += 'delta_x: ' + str(self.x) + '\n'
        string += 'delta_y: ' + str(self.y) + '\n'
        string += 'delta_theta: ' + str(self.theta) + '\n'
        string += 'delta_time: ' + str(self.time) + '\n'
        string += 'vel_curve: ' + str(self.vel_curve)
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
        assert self.vel_curve in self._vel_curve_func
        self.get_vel = self._vel_curve_func[self.vel_curve]

    def default_vel(self, time_duration):
        return (0.0, 0.0, 0.0)

    def square_vel(self, time_duration):
        if time_duration >= self.time:
            self.get_vel = self._vel_curve_func['default']
            return (0.0, 0.0, 0.0)
        return (self.x/self.time, self.y/self.time, 0.0)

    def linear_vel(self, time_duration):
        return (0.0, 0.0, 0.0)
