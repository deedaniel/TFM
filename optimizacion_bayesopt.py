import bayesopt
import push_button.push_button_function as fun
import pickle
import params_opt

TASK_DIR = "push_button/"
# coords_type = 'esfericas'
TASK_NAME = "push_button"  # + "_" + coords_type

n, lb, ub = params_opt.bayesopt_bounds(task=TASK_NAME)

params = {'n_iterations': 300,
          'n_iter_relearn': 10,
          'n_init_samples': 3*n}

listas = []
param_solution = []
n_experimentos = 1

function = fun.PushButton(headless_mode=True)  # Inicializacion

# Coordenadas de la tarea avoid_obstacle
# function.set_coords(coords=coords_type)

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.push_button, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    param_solution.append(x_out)

pickle.dump(listas, open(TASK_DIR + "listas_bayesopt_" + TASK_NAME + ".p", "wb"))
pickle.dump(param_solution, open(TASK_DIR + "solucion_bayesopt_" + TASK_NAME + ".p", "wb"))

function.shutdown()  # Apagado
