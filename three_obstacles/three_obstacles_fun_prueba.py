import three_obstacles.three_obstacles_fun as tof
import pickle

TASK_NAME = "three_obstacles"

file = "solucion_difevol_" + TASK_NAME + ".p"
wp_params = pickle.load(open(file, "rb"))

fun = tof.ThreeObstacles(headless_mode=False)

# wp_params = [0.6, 1.2, 0.25, 0.0]

reward = fun.avoidance_with_waypoints(wp_params[4].x)
print(reward)

fun.shutdown()
