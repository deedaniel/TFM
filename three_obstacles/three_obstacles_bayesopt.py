import bayesopt
import numpy as np
import three_obstacles_fun as pf
import pickle

function = pf.ThreeObstacles()  # Inicializacion

n = 4  # n dimensions
lb = np.array([0.1, -np.pi/2, 0.1, -np.pi/2])
ub = np.array([0.4, np.pi/2, 0.4, np.pi/2])

params = {'n_iterations': 100,
          'n_iter_relearn': 5,
          'n_init_samples': 5*n}

listas = []
n_experimentos = 1

for i in range(n_experimentos):
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.avoidance_with_waypoints, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_bayesopt_3obs.p", "wb"))

function.shutdown()  # Apagado
