import numpy as np
import three_obstacles_fun as tof

fun = tof.ThreeObstacles()

wp_params = np.array([0.3, np.pi / 4.0, np.pi / 2.0, 0.4, np.pi / 2.0, np.pi / 2.0])
reward = fun.avoidance_with_waypoints(wp_params)

fun.shutdown()
