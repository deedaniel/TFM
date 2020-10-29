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

pp_params = np.array([-4.07404621e-03,  2.17105264e-02, -1.84800365e-01,  1.25515623e-01, 1.21314986e-04, -2.19707590e-01])
reward = function.pick_and_place(wp_params[0])
print(reward)

function.shutdown()  # Apagado
