import bayesopt
import slide_object.slide_block_function as fun
import pickle
import params_opt

TASK_DIR = "slide_object/"
# coords_type = 'esfericas'
TASK_NAME = "slide_block"  # + "_" + coords_type

n, lb, ub = params_opt.bayesopt_bounds(task=TASK_NAME)

params = {'n_iterations': 250,
          'n_iter_relearn': 10,
          'n_init_samples': 2*n}

listas = []
param_solution = []
n_experimentos = 5

function = fun.SlideBlock(headless_mode=True)  # Inicializacion

# Coordenadas de la tarea avoid_obstacle
# function.set_coords(coords=coords_type)

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.slide_block, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    param_solution.append(x_out)

pickle.dump(listas, open(TASK_DIR + "listas_bayesopt_" + TASK_NAME + ".p", "wb"))
pickle.dump(param_solution, open(TASK_DIR + "solucion_bayesopt_" + TASK_NAME + ".p", "wb"))

function.shutdown()  # Apagado
