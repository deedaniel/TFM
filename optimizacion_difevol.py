from scipy.optimize import differential_evolution
import numpy as np
import push_button.push_button_function as fun
import pickle

TASK_DIR = "push_button/"

bounds = [(-0.05, 0.05), (-0.05, 0.05), (-0.2, 0.0)]

maxiter = 105//len(bounds)
popsize = 1

listas = []
params_solution = []
n_experimentos = 5

function = fun.PushButton(headless_mode=True)  # Inicializacion

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    result = differential_evolution(function.push_button, bounds,
                                    maxiter=maxiter, popsize=popsize, disp=True)
    print(result)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    params_solution.append(result)

pickle.dump(listas, open(TASK_DIR + "listas_scipy.p", "wb"))
pickle.dump(params_solution, open(TASK_DIR + "solucion_difevol.p", "wb"))

function.shutdown()  # Apagado
