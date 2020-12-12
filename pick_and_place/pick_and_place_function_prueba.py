import pick_and_place.pick_and_place_function as fun
import pickle
import numpy as np

TASK_NAME = "pick_and_place"
VARIATION = '2container'

file = "solucion_difevol_" + TASK_NAME + "_" + VARIATION + ".p"
wp_params = pickle.load(open(file, "rb"))

file2 = "params_solution_" + TASK_NAME + "_" + VARIATION + ".p"
best_params = pickle.load(open(file2, "rb"))

params = best_params[0][0, 300]


function = fun.PickAndPlace(headless_mode=False, variation=VARIATION)  # Inicializacion

pp_params = np.array([0.00506296, -0.00380623, -0.20086191,  0.1014997,   0.00653622, -0.2429435])
reward = function.pick_and_place(pp_params)
print(reward)

function.shutdown()  # Apagado
