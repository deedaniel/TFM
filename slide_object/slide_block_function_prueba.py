import numpy as np
import slide_object.slide_block_function as sb
import pickle

# Definicion de variables para cargar los datos de las optimizaciones
TASK_NAME = "slide_block"
VARIATION = '2block'

# Cargar los datos de las soluciones, los parametros de la politica obtenidos
# file = "solucion_difevol_" + TASK_NAME + "_" + VARIATION + ".p"
# slide_params = pickle.load(open(file, "rb"))

# Cargar los datos obtenidos al hacer la grafica, para cada iteracion los mejores parametros.
file = "params_solution_" + TASK_NAME + "_" + VARIATION + "_sigopt.p"
slide_params = pickle.load(open(file, "rb"))

# Meter parametros de la política a mano
# wp_params = np.array([0.04895007,  0.13835617, -0.19013511,  0.36857607, -0.01570488])

# Inicialización de la tarea
function = sb.SlideBlock(headless_mode=False, variation=VARIATION)  # Inicializacion

# Ejecución de la tarea
reward = function.slide_block(slide_params[0][0, 265, :])
print(reward)

# Apagado de la tarea
function.shutdown()  # Apagado
