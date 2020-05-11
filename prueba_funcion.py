import param_function as pf

function = pf.ParamFunction()  # Inicializacion

radius = [0.27, 0.32]
function.avoidance_task(radius_interval=radius)  # Ejecucion

function.shutdown()  # Apagado
