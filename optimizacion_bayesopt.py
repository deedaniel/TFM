import bayesopt
import pick_and_place.pick_and_place_function as fun
import pickle
import params_opt

TASK_DIR = "pick_and_place/"
# coords_type = 'esfericas'
VARIATION = "2container"
TASK_NAME = "pick_and_place"  # + "_" + coords_type

n, lb, ub = params_opt.bayesopt_bounds(task=TASK_NAME, variation=VARIATION)

params = {'n_iterations': 500,
          'n_iter_relearn': 10,
          'n_init_samples': 3*n}

listas = []
param_solution = []
n_experimentos = 5

function = fun.PickAndPlace(headless_mode=True, variation=VARIATION)  # Inicializacion

# Coordenadas de la tarea avoid_obstacle
# function.set_coords(coords=coords_type)

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.pick_and_place, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    param_solution.append(x_out)

pickle.dump(listas, open(TASK_DIR + "listas_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))
pickle.dump(param_solution, open(TASK_DIR + "solucion_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))

function.shutdown()  # Apagado
