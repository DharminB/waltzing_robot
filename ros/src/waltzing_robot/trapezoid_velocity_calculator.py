from __future__ import print_function

class TrapezoidVelocityCalculator(object):

    """Calculate desired velocity when following things are provided using sampling
       1) total distance to be travelled (meters)
       2) total travel time (seconds)
       3) max acceleration (meters per second squared)
       4) max decceleration (meters per second squared)
       5) max allowed velocity (meters per second)
       5) min allowed velocity (meters per second) (should be > 0.0)
       6) number of samples per iteration
    """

    def __init__(self, max_acc=5.0, max_dec=5.0, max_vel=5.0, min_vel=0.0001, num_of_sample=10):
        self.a_max = max_acc
        self.d_max = max_dec
        self.v_max = max_vel
        self.v_min = min_vel
        self.v_tolerance = self.v_min
        self.num_of_sample = num_of_sample

    def __str__(self):
        string = ""
        string += 'max_vel: ' + str(self.v_max) + '\n'
        string += 'min_vel: ' + str(self.v_min) + '\n'
        string += 'max_acc: ' + str(self.a_max) + '\n'
        string += 'max_dec: ' + str(self.d_max) + '\n'
        string += 'num_of_sample: ' + str(self.num_of_sample) + '\n'
        string += 'vel tolerance: ' + str(self.v_tolerance) + '\n'
        return string

    def calc_desired_vel(self, distance=1.0, time=1.0):
        """
        Calculates desired velocity for travelling given distance in a specified
        time by sampling velocity iteratively.
        
        :distance: float
        :time: float

        :returns: list of dict (each dict has keys 'vel', 't_acc', 't_const_vel' and 't_dec')

        """
        if distance == 0.0 and time > 0.0:
            return [{'vel': 0.0, 't_acc': 0.0, 't_const_vel': time, 't_dec': 0.0}]
        vel_range_queue = [{'v_start': self.v_min, 'v_stop': self.v_max}]
        vel_answer_candidates = []

        while len(vel_range_queue) > 0:
            vel_range = vel_range_queue.pop(0)
            data = TrapezoidVelocityCalculator.sample_between_vel_range(self.a_max,
                                                                        self.d_max,
                                                                        distance,
                                                                        vel_range['v_start'],
                                                                        vel_range['v_stop'],
                                                                        self.num_of_sample)
            # print(vel_range)
            # for i in data:
            #     print(i)

            for i in range(len(data)-1):
                counter = 0
                counter += 1 if (data[i][1] - time) > 0 else 0
                counter += 1 if (data[i+1][1] - time) > 0 else 0
                if counter == 1:
                    vel_range = {'v_start': data[i][0], 'v_stop': data[i+1][0]}
                    if vel_range['v_stop'] - vel_range['v_start'] < self.v_tolerance:
                        vel_answer_candidates.append(vel_range['v_stop'])
                    else:
                        vel_range_queue.append(vel_range)

        desired_vel_list = []
        for vel in vel_answer_candidates:
            t_A, t_B, t_C = TrapezoidVelocityCalculator.get_travel_times(
                                    self.a_max, self.d_max, distance, vel)
            desired_vel = {'vel': vel, 't_acc': t_A, 't_const_vel': t_B, 't_dec': t_C}
            desired_vel_list.append(desired_vel)

        return desired_vel_list

    @staticmethod
    def sample_between_vel_range(a_max, d_max, d, v_start, v_stop, num_of_samples):
        """
        Creates velocity sample between v_start and v_stop at regular interval.
        Returns list of tuple containing sampled velocities and corresponding travel times

        :a_max: float
        :d_max: float
        :d: float
        :v_start: float
        :v_stop: float
        :num_of_samples: int

        :returns: list of tuples (each tuple containing 2 float values)

        """
        v_step = (v_stop-v_start)/num_of_samples
        vel_time_data = []
        for i in range(0, num_of_samples+1):
            v_des = v_start + (i*v_step)
            calc_times = TrapezoidVelocityCalculator.get_travel_times(a_max, d_max, d, v_des)
            if calc_times is None:
                continue
            calc_total_time = sum(calc_times)
            vel_time_data.append((v_des, calc_total_time))
        return vel_time_data

    @staticmethod
    def get_travel_times(a_max, d_max, d, v_des):
        """
        Calculate the travel time when desired velocity (v_des) is used to travel
        distance (d). The distance travelled is partitioned into 3 parts
            A) part with max acceleration
            B) part with constant velocity (v_des)
            C) part with max decceleration
        d_A is distance covered in part A and t_A is time taken for part A

        Returns None if any of distances travelled or time taken is non positive
            Except for part B where the distance and time can be zero

        :a_max: float
        :d_max: float
        :d: float
        :v_des: float

        :returns: tuple of float or None
        """
        t_A = v_des/a_max
        t_C = v_des/d_max
        d_A = (0.5) * a_max * (t_A**2)
        d_C = (v_des * t_C) - ((0.5) * d_max * (t_C**2))
        d_B = d - d_A - d_C
        t_B = d_B / v_des
        if d_A > 0 and d_B >= 0 and d_C > 0 and t_A > 0 and t_B >= 0 and t_C > 0:
            return (t_A, t_B, t_C)
        else:
            return None
