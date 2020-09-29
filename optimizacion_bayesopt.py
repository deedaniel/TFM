import bayesopt
import numpy as np
import push_button.push_button_function as fun
import pickle

TASK_DIR = "push_button/"

n = 3  # n dimensions
lb = np.array([-0.05, -0.05, -0.20])
ub = np.array([0.05, 0.05, 0.0])

params = {'n_iterations': 100,
          'n_iter_relearn': 10,
          'n_init_samples': 2*n}

listas = []
param_solution = []
n_experimentos = 5

function = fun.PushButton(headless_mode=True)  # Inicializacion

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.push_button, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    param_solution.append(x_out)

pickle.dump(listas, open(TASK_DIR + "listas_bayesopt.p", "wb"))
pickle.dump(param_solution, open(TASK_DIR + "solucion_bayesopt.p", "wb"))

function.shutdown()  # Apagado
