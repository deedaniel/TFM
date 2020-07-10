import bayesopt
import numpy as np
import param_function as pf
import pickle

function = pf.ParamFunction()  # Inicializacion

params = {'n_iterations': 20, 'n_iter_relearn': 5, 'n_init_samples': 2}

n = 1  # n dimensions
lb = np.array([0.27])
ub = np.array([0.32])

mvalue, x_out, error = bayesopt.optimize(function.avoidance_tray_circular, n, lb, ub, params)

print("Result", mvalue, "at", x_out)

listas = function.return_lists()

pickle.dump(listas, open("listas_bayesopt_tray_circular.p", "wb"))

function.shutdown()  # Apagado
