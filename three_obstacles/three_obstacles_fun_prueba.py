import numpy as np
import three_obstacles_fun as tof

fun = tof.ThreeObstacles()

wp_params = np.array([0.2, +np.pi/10, 0.4, 0.0])
reward = fun.avoidance_with_waypoints(wp_params)
print(reward)

fun.shutdown()
