import numpy as np
import slide_block_function as sb

function = sb.SlideBlock()  # Inicializacion

wp_params = np.array([0, 0.05, -0.15, 0.2, 0])
reward = function.slide_block(wp_params)
print(reward)

function.shutdown()  # Apagado