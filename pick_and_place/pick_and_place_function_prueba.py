import pick_and_place.pick_and_place_function as fun
import pickle
import numpy as np

TASK_NAME = "pick_and_place"
# coords_type = "esfericas"
VARIATION = '2container'

file = "solucion_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p"
wp_params = pickle.load(open(file, "rb"))
print(wp_params[0])

function = fun.PickAndPlace(headless_mode=False, variation=VARIATION)  # Inicializacion

pp_params = np.array([0.000945996,0.0132727,-0.221771,0.0958262,0.0281767,-0.195166])
reward = function.pick_and_place(pp_params)
print(reward)

function.shutdown()  # Apagado
