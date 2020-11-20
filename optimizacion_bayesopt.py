import bayesopt
import avoid_obstacle.avoid_obstacle_function as fun
import pickle
import params_opt

TASK_DIR = "avoid_obstacle/"
VARIATION = 'esfericas'
TASK_NAME = "avoid_obstacle"

n, lb, ub = params_opt.bayesopt_bounds(task=TASK_NAME, coords=VARIATION)

params = {'n_iterations': 200,
          'n_iter_relearn': 10,
          'n_init_samples': 5*n}

listas = []
param_solution = []
n_experimentos = 5

function = fun.AvoidObstacle(headless_mode=True)  # Inicializacion

# Coordenadas de la tarea avoid_obstacle
function.set_coords(coords=VARIATION)

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    mvalue, x_out, error = bayesopt.optimize(function.tray_with_waypoints, n, lb, ub, params)
    print("Result", mvalue, "at", x_out)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    param_solution.append(x_out)

pickle.dump(listas, open(TASK_DIR + "listas_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))
pickle.dump(param_solution, open(TASK_DIR + "solucion_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))

function.shutdown()  # Apagado
