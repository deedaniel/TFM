import bayesopt
import numpy as np
import push_button_function as sbf
import pickle

function = sbf.PushButton()  # Inicializacion

n = 3  # n dimensions
lb = np.array([-0.3, -0.3, -0.1])
ub = np.array([0.3, 0.3, 0.1])

params = {'n_iterations': 500,
          'n_iter_relearn': 10,
          'n_init_samples': 2*n}

listas = []
n_experimentos = 5

for i in range(n_experimentos):
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.push_button, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_bayesopt_pushbutton.p", "wb"))

function.shutdown()  # Apagado
