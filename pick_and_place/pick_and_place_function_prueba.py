import pick_and_place.pick_and_place_function as fun
import pickle
import numpy as np

TASK_NAME = "pick_and_place"
# coords_type = "esfericas"
VARIATION = '1container'

# file = "solucion_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p"
# wp_params = pickle.load(open(file, "rb"))
# print(wp_params[0])

function = fun.PickAndPlace(headless_mode=False, variation=VARIATION)  # Inicializacion

pp_params = np.array([-0.06991656, -0.00729467, -0.24195188, -0., -0.0056952,  -0.14584554])
reward = function.pick_and_place(pp_params)
print(reward)

function.shutdown()  # Apagado
