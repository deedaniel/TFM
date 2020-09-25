import numpy as np
import avoid_obstacle.param_function as pf
import pickle

file = "param solucion bayesopt.p"
params_solution = pickle.load(open(file, "rb"))

print(params_solution)


