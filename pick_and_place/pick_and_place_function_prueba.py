import numpy as np
import pick_and_place_function as ppf

function = ppf.PickAndPlace()  # Inicializacion

wp_params = np.array([0.00618781,-0.00204216,-0.163625,0.020944,-0.0560744,-0.102735])
reward = function.pick_and_place(wp_params)
print(reward)

function.shutdown()  # Apagado
