import numpy as np
import avoid_obstacle.avoid_obstacle_function as fun
import math
import pickle

VARIATION = "1obstacle"
COORDS = "cartesianas"
TASK_NAME = "avoid_obstacle" + "_" + VARIATION + "_" + COORDS

file = "solucion_difevol_" + TASK_NAME + ".p"
params_solution = pickle.load(open(file, "rb"))

wp_params = np.array([0.0,  0.25,  0.3, 0.0,  0.25, 0.0])
# wp_params = np.array([0.3, math.pi / 4.0, math.pi / 2.0, 0.4, math.pi / 2.0, math.pi / 2.0])

function = fun.AvoidObstacle(headless_mode=False, variation=VARIATION, coords=COORDS)  # Inicializacion
reward = function.avoid1obstacle(wp_params)

print(reward)

function.shutdown()  # Apagado
