from scipy.optimize import differential_evolution
import numpy as np
import slide_block_function as sbf
import pickle

function = sbf.SlideBlock()  # Inicializacion

bounds = [(-0.2, 0.2), (0, 0.2), (-0.25, 0.0), (0.05, 0.4), (-np.pi/4, np.pi/4)]

maxiter = 250
popsize = 1

listas = []
n_experimentos = 5

for i in range(n_experimentos):
    function.clean_lists()
    result = differential_evolution(function.slide_block, bounds, maxiter=maxiter, popsize=popsize)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_scipy_pickandplace.p", "wb"))

function.shutdown()  # Apagado
