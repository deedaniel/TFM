import numpy as np
import avoid_obstacle.param_function as pf
import math

function = pf.ParamFunction()  # Inicializacion
# function = sbf.SlideBlockFunction()

# radius = [0.27, 0.32]
# function.avoidance_brute_force(radius_interval=radius)  # Ejecucion

wp_params = np.array([0.11279,-0.165146,0.34462,0.03865,0.375,0.327896])
reward = function.tray_with_waypoints(wp_params)
print(reward)

# wp_params = np.array([0.3, math.pi / 4.0, math.pi / 2.0, 0.4, math.pi / 2.0, math.pi / 2.0])
# reward = function.tray_with_waypoints(wp_params, coords='esfericas')

# slide_params = np.array([0.3, 0])
# reward = function.slide_block(slide_params)
# print(reward)

function.shutdown()  # Apagado
