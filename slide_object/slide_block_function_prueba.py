import numpy as np
import slide_object.slide_block_function as sb
import pickle

TASK_NAME = "slide_block"
# coords_type = "esfericas"
VARIATION = '2block'

# file = "solucion_difevol_" + TASK_NAME + "_" + VARIATION + ".p"
# slide_params = pickle.load(open(file, "rb"))

wp_params = np.array([0.04895007,  0.13835617, -0.19013511,  0.36857607, -0.01570488])

function = sb.SlideBlock(headless_mode=True, variation=VARIATION)  # Inicializacion
reward = function.slide_block(wp_params)
print(reward)

function.shutdown()  # Apagado
