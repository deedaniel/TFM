from scipy.optimize import differential_evolution
import pick_and_place.pick_and_place_function as fun
import pickle
import params_opt

TASK_DIR = "pick_and_place/"
# coords_type = 'esfericas'
TASK_NAME = "pick_and_place"  # + "_" + coords_type
VARIATION = "2container"

bounds = params_opt.difevol_bounds(task=TASK_NAME, variation=VARIATION)

popsize = 10
maxiter = 300//(len(bounds)*popsize)

listas = []
params_solution = []
n_experimentos = 5

function = fun.PickAndPlace(headless_mode=True, variation=VARIATION)  # Inicializacion

# Coordenadas de la tarea avoid_obstacle
# function.set_coords(coords=coords_type)

for i in range(n_experimentos):
    print(i)
    function.clean_lists()
    result = differential_evolution(function.pick_and_place, bounds,
                                    maxiter=maxiter, popsize=popsize, disp=True,
                                    polish=False)
    print(result)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)
    params_solution.append(result)

pickle.dump(listas, open(TASK_DIR + "listas_difevol_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))
pickle.dump(params_solution, open(TASK_DIR + "solucion_difevol_" + TASK_NAME + "_" + VARIATION + ".p", "wb"))

function.shutdown()  # Apagado
