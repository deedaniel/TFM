from scipy.optimize import differential_evolution
import param_function as pf
import pickle

function = pf.ParamFunction()  # Inicializacion

bounds = [(0.27, 0.32)]

result = differential_evolution(function.avoidance_tray_circular, bounds)
print(result)

listas = function.return_lists()

pickle.dump(listas, open("listas_scipy_tray_circular.p", "wb"))

function.shutdown()  # Apagado
