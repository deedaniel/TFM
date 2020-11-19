import pick_and_place.pick_and_place_function as fun
import pickle
import numpy as np

TASK_NAME = "pick_and_place"
# coords_type = "esfericas"
VARIATION = '2container'

# file = "solucion_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p"
# wp_params = pickle.load(open(file, "rb"))
# print(wp_params[0])

function = fun.PickAndPlace(headless_mode=False, variation=VARIATION)  # Inicializacion

pp_params = np.array([-0.0120154,0.0062088,-0.217875,0.0937116,-0.00943356,-0.185821])
reward = function.pick_and_place(pp_params)
print(reward)

function.shutdown()  # Apagado
