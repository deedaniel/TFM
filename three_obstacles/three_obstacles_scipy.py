from scipy.optimize import differential_evolution
import numpy as np
import three_obstacles.three_obstacles_fun as sbf
import pickle

function = sbf.ThreeObstacles(headless_mode=False)  # Inicializacion

bounds = [(0.1, 0.4), (-np.pi/2, np.pi/2), (0.1, 0.4), (-np.pi/2, np.pi/2)]

maxiter = 280//len(bounds)
popsize = 1

listas = []
n_experimentos = 5

for i in range(n_experimentos):
    function.clean_lists()
    result = differential_evolution(function.avoidance_with_waypoints, bounds,
                                    maxiter=maxiter, popsize=popsize, disp=True)
    print(result)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_scipy_3obs.p", "wb"))

function.shutdown()  # Apagado
