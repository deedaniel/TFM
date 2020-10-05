import bayesopt
import numpy as np
import avoid_obstacle.avoid_obstacle_function as pf
import pickle

function = pf.AvoidObstacle(headless_mode=True)  # Inicializacion

n = 6  # n dimensions
lb = np.array([0.1, 0, 0, 0.1, 0, 0])
ub = np.array([0.5, np.pi, np.pi, 0.5, np.pi, np.pi])

params = {'n_iterations': 250,
          'n_iter_relearn': 10,
          'n_init_samples': 2*n}

function.set_coords('esfericas')

listas = []
params_solution = []
n_experimentos = 5

for i in range(n_experimentos):
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.tray_with_waypoints, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    params_solution.append(x_out)

pickle.dump(listas, open("listas_bayesopt_waypoints_esfericas_prueba.p", "wb"))
pickle.dump(params_solution, open("solucion_bayesopt_waypoints_esfericas_prueba.p", "wb"))

function.shutdown()  # Apagado
