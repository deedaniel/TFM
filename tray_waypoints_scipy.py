from scipy.optimize import differential_evolution

import param_function as pf

function = pf.ParamFunction()  # Inicializacion

bounds = [(-0.4, 0.4), (-0.375, 0.375), (-0.1, 0.4), (-0.4, 0.4), (-0.375, 0.375), (-0.1, 0.4)]

result = differential_evolution(function.tray_with_waypoints, bounds)
print(result)

function.shutdown()  # Apagado
