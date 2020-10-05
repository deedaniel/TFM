import numpy as np
import avoid_obstacle.avoid_obstacle_function as pf
import pickle

file = "listas_difevol_avoid_obstacle.p"
listas = pickle.load(open(file, "rb"))

array = np.asarray(listas)
max_it = 0

for i in range(len(listas)):
    if len(listas[i].list_of_rewards) > max_it:
        max_it = len(listas[i].list_of_rewards)

print(max_it)
