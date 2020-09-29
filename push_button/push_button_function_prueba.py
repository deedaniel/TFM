import numpy as np
import push_button.push_button_function as fun

function = fun.PushButton(headless_mode=True)  # Inicializacion

push_params = np.array([0, 0, 0])
reward = function.push_button(push_params)
print(reward)

function.shutdown()  # Apagado
