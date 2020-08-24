import numpy as np
import push_button_function as pbf

function = pbf.PushButton()  # Inicializacion

push_params = np.array([0, 0, 0.01])
reward = function.push_button(push_params)
print(reward)

function.shutdown()  # Apagado
