import numpy as np
import three_obstacles.three_obstacles_fun as tof
import pickle

file = "param solucion bayesopt.p"
wp_params = pickle.load(open(file, "rb"))

fun = tof.ThreeObstacles(headless_mode=False)

reward = fun.avoidance_with_waypoints(wp_params[1])
print(reward)

fun.shutdown()
