import numpy as np
import three_obstacles_fun as tof

fun = tof.ThreeObstacles()

wp_params = np.array([0.25,1.0472,0.25,0])
reward = fun.avoidance_with_waypoints(wp_params)
print(reward)

fun.shutdown()
