from scipy.optimize import differential_evolution
import avoid_obstacle.avoid_obstacle_function as fun
import pickle
import params_opt

TASK_DIR = "avoid_obstacle/"
VARIATION = 'esfericas'
TASK_NAME = "avoid_obstacle"

bounds = params_opt.difevol_bounds(task=TASK_NAME, coords=VARIATION)

popsize = 5
maxiter = 200//(len(bounds)*popsize)

listas = []
params_solution = []
n_experimentos = 5

function = fun.AvoidObstacle(headless_mode=True)  # Inicializacion

# Coordenadas de la tarea avoid_obstacle
function.set_coords(coords=VARIATION)

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    result = differential_evolution(function.tray_with_waypoints, bounds,
                                    maxiter=maxiter, popsize=popsize, disp=True, polish=False)
    print(result)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    params_solution.append(result)

pickle.dump(listas, open(TASK_DIR + "listas_difevol_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))
pickle.dump(params_solution, open(TASK_DIR + "solucion_difevol_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))

function.shutdown()  # Apagado
