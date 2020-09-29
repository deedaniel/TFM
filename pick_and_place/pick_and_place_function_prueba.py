import pick_and_place.pick_and_place_function as ppf
import pickle

file = "solutions_scipy_esf_wp.p"
wp_params = pickle.load(open(file, "rb"))

function = ppf.PickAndPlace()  # Inicializacion

reward = function.pick_and_place(wp_params[0])
print(reward)

function.shutdown()  # Apagado
