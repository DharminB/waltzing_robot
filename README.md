# Waltzing robot

Make a planar mobile robot 
- simple waltz dance or
- follow choreography using waypoints

## Dependencies
- [ropod_sim_model](https://github.com/ropod-project/ropod_sim_model)

## Waypoint choreography (under development)

### Trajectory using linear velocity
![rvizImage](docs/waltz_trajectory.png)
![rvizImage](docs/square_clockwise_linear.png)

## Simple waltz
```
v = V_MAX * (RADIUS ^ (t / T_MAX))

where
    V_MAX = 4
    RADIUS = 0.01
    T_MAX = 2.0
```
![plot](docs/plot.png)

The plot can also be visualised [here](https://www.desmos.com/calculator).

A screen recording video can be viewed [here](docs/screen_recording.mp4).

