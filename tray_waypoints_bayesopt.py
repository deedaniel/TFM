import bayesopt
import numpy as np
import param_function as pf
import matplotlib.pyplot as plt
import pickle

function = pf.ParamFunction()  # Inicializacion

n = 6  # n dimensions
lb = np.array([-0.3, -0.375, -0.10, -0.3, -0.375, -0.10])
ub = np.array([0.3, 0.375, 0.4, 0.3, 0.375, 0.4])

params = {'n_iterations': 10,
          'n_iter_relearn': 10,
          'n_init_samples': 2*n}

listas = []
for i in range(5):
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.tray_with_waypoints, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_bayesopt_waypoints.p", "wb"))

function.shutdown()  # Apagado
