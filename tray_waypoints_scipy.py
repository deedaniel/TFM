from scipy.optimize import differential_evolution
import param_function as pf
import pickle

function = pf.ParamFunction()  # Inicializacion

bounds = [(-0.4, 0.4), (-0.375, 0.375), (-0.1, 0.4), (-0.4, 0.4), (-0.375, 0.375), (-0.1, 0.4)]

maxiter = 1000
popsize = 1

result = differential_evolution(function.tray_with_waypoints, bounds, maxiter=maxiter, popsize=popsize)
print('Result:', result.x)
print('Function:', result.fun)

listas = function.return_lists()

pickle.dump(listas, open("listas_scipy_waypoints.p", "wb"))

function.shutdown()  # Apagado
