import three_obstacles.three_obstacles_fun as tof
import pickle

# file = "solutions_scipy_esf_wp.p"
# wp_params = pickle.load(open(file, "rb"))

fun = tof.ThreeObstacles(headless_mode=False)

wp_params = [0.33, 0.45, 0.25, 0.0]

reward = fun.avoidance_with_waypoints(wp_params)
print(reward)

fun.shutdown()
