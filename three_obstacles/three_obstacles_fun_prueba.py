import three_obstacles.three_obstacles_fun as tof
import pickle

file = "solutions_scipy_esf_wp.p"
wp_params = pickle.load(open(file, "rb"))

fun = tof.ThreeObstacles(headless_mode=True)

reward = fun.avoidance_with_waypoints(wp_params[0])
print(reward)

fun.shutdown()
