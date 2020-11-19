import pick_and_place.pick_and_place_function as fun
import pickle
import numpy as np

TASK_NAME = "pick_and_place"
VARIATION = '2container'

file = "solucion_difevol_" + TASK_NAME + "_" + VARIATION + ".p"
wp_params = pickle.load(open(file, "rb"))

function = fun.PickAndPlace(headless_mode=False, variation=VARIATION)  # Inicializacion

pp_params = np.array([-0.06991656, -0.00729467, -0.24195188, -0., -0.0056952,  -0.14584554])
reward = function.pick_and_place(wp_params[4].x)
print(reward)

function.shutdown()  # Apagado
