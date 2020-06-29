from scipy.optimize import differential_evolution

import param_function as pf

function = pf.ParamFunction()  # Inicializacion

bounds = [(0.27, 0.32)]

result = differential_evolution(function.avoidance_tray_circular, bounds)
print(result)

function.shutdown()  # Apagado
