import numpy as np
import pickle

file1 = "listas_scipy_waypoints_esfericas.p"
resultados1 = pickle.load(open(file1, "rb"))

param_list = []

n_iteraciones = len(resultados1[0].list_of_rewards)
n_experimentos = len(resultados1)

best_param = []

for i in range(n_experimentos):
    for j in range(n_iteraciones):
        param = [resultados1[i].list_of_parameters[6*j],
                 resultados1[i].list_of_parameters[6*j+1],
                 resultados1[i].list_of_parameters[6*j+2],
                 resultados1[i].list_of_parameters[6*j+3],
                 resultados1[i].list_of_parameters[6*j+4],
                 resultados1[i].list_of_parameters[6*j+5]]
        param_list.append(param)
    resultados1[i].list_of_parameters = param_list
    best_param.append(param_list[int(np.argmin(resultados1[i].list_of_rewards))])

pickle.dump(resultados1, open("listas_scipy_esf_wp.p", "wb"))
pickle.dump(best_param, open("solutions_scipy_esf_wp.p", "wb"))
