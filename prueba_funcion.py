import numpy as np
import param_function as pf

function = pf.ParamFunction()  # Inicializacion

#radius = [0.27, 0.32]
#function.avoidance_brute_force(radius_interval=radius)  # Ejecucion

wp_params = np.array([0.02682544,  0.02903595,  0.3408096,   0.11211002, -0.31589305, -0.09995094])
function.tray_with_waypoints(wp_params)

function.shutdown()  # Apagado
