import numpy as np
import slide_object.slide_block_function as sb
import pickle

TASK_NAME = "slide_object"
# coords_type = "esfericas"
VARIATION = '2block'

# file = "solucion_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p"
# slide_params = pickle.load(open(file, "rb"))

wp_params = np.array([0.03, 0.055, -0.15, 0.2, - np.pi / 6])

function = sb.SlideBlock(headless_mode=False, variation=VARIATION)  # Inicializacion
reward = function.slide_block(wp_params)
print(reward)

function.shutdown()  # Apagado
