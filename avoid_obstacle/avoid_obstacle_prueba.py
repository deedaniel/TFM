import numpy as np
import avoid_obstacle.avoid_obstacle_function as fun
import math
import pickle

COORDS = "cartesianas"
TASK_NAME = "avoid_obstacle" + "_" + COORDS

file = "solucion_difevol_" + TASK_NAME + ".p"
params_solution = pickle.load(open(file, "rb"))

# radius = [0.27, 0.32]
wp_params = np.array([0.0,  0.25,  0.3, 0.0,  0.25, 0.0])
# wp_params = np.array([0.3, math.pi / 4.0, math.pi / 2.0, 0.4, math.pi / 2.0, math.pi / 2.0])

function = fun.AvoidObstacle(headless_mode=False)  # Inicializacion
function.set_coords(coords=COORDS)
# function.avoidance_brute_force(radius_interval=radius)  # Ejecucion
reward = function.tray_with_waypoints(wp_params)

print(reward)

function.shutdown()  # Apagado
