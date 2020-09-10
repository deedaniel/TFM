from scipy.optimize import differential_evolution
import pick_and_place_function as pf
import pickle

function = pf.PickAndPlace()  # Inicializacion

bounds = [(-0.15, 0.15), (-0.15, 0.15), (-0.25, -0.15), (-0.1, 0.1), (-0.1, 0.1), (-0.25, -0.15)]

maxiter = 200
popsize = 1

listas = []
n_experimentos = 5

for i in range(n_experimentos):
    function.clean_lists()
    result = differential_evolution(function.pick_and_place, bounds, maxiter=maxiter, popsize=popsize)
    print(result)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_scipy_pickandplace.p", "wb"))

function.shutdown()  # Apagado
