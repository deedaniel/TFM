import bayesopt
import numpy as np
import param_function as pf
import matplotlib.pyplot as plt
import pickle

function = pf.ParamFunction()  # Inicializacion

params = {'n_iterations': 500, 'n_iter_relearn': 5, 'n_init_samples': 2}

n = 6  # n dimensions
lb = np.array([-0.3, -0.375, -0.10, -0.3, -0.375, -0.10])
ub = np.array([0.3, 0.375, 0.4, 0.3, 0.375, 0.4])

mvalue, x_out, error = bayesopt.optimize(function.tray_with_waypoints, n, lb, ub, params)

print("Result", mvalue, "at", x_out)

listas = function.return_lists()

pickle.dump(listas, open("save_listas.p", "wb"))

function.shutdown()  # Apagado
