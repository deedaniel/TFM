import numpy as np
import param_function as pf
import math
import slide_block_function as sbf

#function = pf.ParamFunction()  # Inicializacion
function = sbf.SlideBlockFunction()

#radius = [0.27, 0.32]
#function.avoidance_brute_force(radius_interval=radius)  # Ejecucion

#wp_params = np.array([-0.299974,0.37498,0.399992,-0.299942,-0.374927,0.316419])
#reward = function.tray_with_waypoints(wp_params, coords='cartesianas')
#print(reward)

#wp_params = np.array([0.3, math.pi / 4.0, math.pi / 2.0, 0.4, math.pi / 2.0, math.pi / 2.0])
#reward = function.tray_with_waypoints(wp_params, coords='esfericas')

slide_params = np.array([0.3, 0])
reward = function.slide_block(slide_params)
print(reward)

function.shutdown()  # Apagado
