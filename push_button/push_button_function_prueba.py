import numpy as np
import push_button.push_button_function as fun
import pickle

TASK_NAME = "push_button"
# coords_type = "esfericas"
VARIATION = '1button'

file = "solucion_bayesopt_" + TASK_NAME + "_" + VARIATION + ".p"
wp_params = pickle.load(open(file, "rb"))

function = fun.PushButton(headless_mode=False, variation=VARIATION)  # Inicializacion

push_params = np.array([0.0685, 0.0, -0.1187, 0.0, np.pi / 6, 0.0])
# push_params = np.array([-0.0350716,-0.00212258,-0.178735,-0.361004,0.844788,-0.547551])
reward = function.push_button(push_params)
print(reward)

function.shutdown()  # Apagado
