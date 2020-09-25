from scipy.optimize import differential_evolution
import numpy as np
import slide_object.slide_block_function as sbf
import pickle

bounds = [(-0.2, 0.2), (0, 0.2), (-0.25, 0.0), (0.05, 0.4), (-np.pi/4, np.pi/4)]

maxiter = 300//len(bounds)
popsize = 1

listas = []
n_experimentos = 5

function = sbf.SlideBlock(headless_mode=True)  # Inicializacion

for i in range(n_experimentos):
    print('Experimento nº: ' + str(i))
    function.clean_lists()
    result = differential_evolution(function.slide_block, bounds, maxiter=maxiter, popsize=popsize, disp=True)
    print(result)
    listas_optimizacion = function.return_lists()
    listas.append(listas_optimizacion)

pickle.dump(listas, open("listas_scipy_slideblock.p", "wb"))

function.shutdown()  # Apagado
