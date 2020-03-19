#! /usr/bin/env python

import os
import yaml
import matplotlib.pyplot as plt

FILE_PATH = '/tmp/waltz_vel_data.yaml'

def get_vel_data():
    try:
        with open(FILE_PATH, 'r') as file_obj:
            data = yaml.safe_load(file_obj)
            return data
    except Exception as e:
        return None

def plot_vel_data(data):
    x = [round(p[0], 3) for p in data]
    y = [round(p[1], 3) for p in data]
    theta = [round(p[2], 3) for p in data]
    time = range(len(data))
    plt.plot(time, x, 'r-')
    plt.plot(time, y, 'b-')
    plt.plot(time, theta, 'k-')
    plt.legend(['X', 'Y', 'Theta'])
    plt.show()

def main():
    data = get_vel_data()
    if data is None:
        print('No data')
        return
    plot_vel_data(data)

if __name__ == "__main__":
    main()
