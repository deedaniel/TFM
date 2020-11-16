from scipy.optimize import differential_evolution
import three_obstacles.three_obstacles_fun as fun
import pickle
import params_opt

TASK_DIR = "three_obstacles/"
# coords_type = 'esfericas'
TASK_NAME = "three_obstacles"  # + "_" + coords_type

bounds = params_opt.difevol_bounds(task=TASK_NAME)

maxiter = 300//len(bounds)
popsize = 10

listas = []
params_solution = []
n_experimentos = 5

function = fun.ThreeObstacles(headless_mode=True)  # Inicializacion

# Coordenadas de la tarea avoid_obstacle
# function.set_coords(coords=coords_type)

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    result = differential_evolution(function.avoidance_with_waypoints, bounds,
                                    maxiter=maxiter, disp=True, polish=False)
    print(result)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    params_solution.append(result)

pickle.dump(listas, open(TASK_DIR + "listas_difevol_" + TASK_NAME + ".p", "wb"))
pickle.dump(params_solution, open(TASK_DIR + "solucion_difevol_" + TASK_NAME + ".p", "wb"))

function.shutdown()  # Apagado
