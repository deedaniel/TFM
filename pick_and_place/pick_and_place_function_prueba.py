import pick_and_place.pick_and_place_function as ppf
import pickle
import numpy as np

file = "solutions_scipy_esf_wp.p"
wp_params = pickle.load(open(file, "rb"))

pickplace_params = np.array([0, 0, 0, 0, 0, 0])
function = ppf.PickAndPlace(headless_mode=False)  # Inicializacion

reward = function.pick_and_place(wp_params[0])
print(reward)

function.shutdown()  # Apagado
