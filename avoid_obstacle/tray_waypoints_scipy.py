from scipy.optimize import differential_evolution
import numpy as np
import param_function as pf
import pickle

function = pf.ParamFunction()  # Inicializacion

bounds = [(0.1, 0.5), (0.0, np.pi), (0.0, np.pi), (0.1, 0.5), (0.0, np.pi), (0.0, np.pi)]

function.set_coords('esfericas')

maxiter = 500
popsize = 1

listas = []
n_experimentos = 5

for i in range(n_experimentos):
    function.clean_lists()
    result = differential_evolution(function.tray_with_waypoints, bounds, maxiter=maxiter, popsize=popsize)
    print('Result:', result.x)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_scipy_waypoints_esfericas.p", "wb"))

function.shutdown()  # Apagado
