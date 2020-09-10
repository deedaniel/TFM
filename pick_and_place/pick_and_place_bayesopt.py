import bayesopt
import numpy as np
import pick_and_place_function as pf
import pickle

function = pf.PickAndPlace()  # Inicializacion

n = 6  # n dimensions
lb = np.array([-0.15, -0.15, -0.25, -0.1, -0.1, -0.25])
ub = np.array([0.15, 0.15, -0.14, 0.1, 0.1, -0.14])

params = {'n_iterations': 250,
          'n_iter_relearn': 5,
          'n_init_samples': 2*n,
          'l_type': 'L_MCMC'}

listas = []
n_experimentos = 5

for i in range(n_experimentos):
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.pick_and_place, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_bayesopt_pickandplace.p", "wb"))

function.shutdown()  # Apagado
