import numpy as np
import pick_and_place_function as ppf

function = ppf.PickAndPlace()  # Inicializacion

wp_params = np.array([0, 0, 0, 0, 0, 0.1])
reward = function.pick_and_place(wp_params)
print(reward)

function.shutdown()  # Apagado
