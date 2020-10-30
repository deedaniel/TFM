import numpy as np
import push_button.push_button_function as fun
import pickle

TASK_NAME = "push_button"
# coords_type = "esfericas"
variation = '2button'

file = "solucion_bayesopt_" + TASK_NAME + ".p"
wp_params = pickle.load(open(file, "rb"))

function = fun.PushButton(headless_mode=False, variation=variation)  # Inicializacion

push_params = np.array([0.103, 0.0, -0.0987, 0.0, np.pi / 6, 0.0])
# push_params = np.array([-0.0350716,-0.00212258,-0.178735,-0.361004,0.844788,-0.547551])
reward = function.push_button(push_params)
print(reward)

function.shutdown()  # Apagado
