from scipy.optimize import differential_evolution
import pick_and_place_function as pf
import pickle

function = pf.PickAndPlace()  # Inicializacion

bounds = [(-0.4, 0.4), (-0.375, 0.375), (-0.1, 0.4), (-0.4, 0.4), (-0.375, 0.375), (-0.1, 0.4)]

maxiter = 500
popsize = 1

listas = []
n_experimentos = 5

for i in range(n_experimentos):
    function.clean_lists()
    result = differential_evolution(function.tray_with_waypoints, bounds, maxiter=maxiter, popsize=popsize)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_scipy_pickandplace.p", "wb"))

function.shutdown()  # Apagado