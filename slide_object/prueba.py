import numpy as np
import pickle

TASK_NAME = "slide_block"  # + "_" + coords_type
VARIATION = "1block"

file = "listas_difevol_slide_block_1block.p"
listas = pickle.load(open(file, "rb"))

file1 = "listas_difevol_slide_block_1block.p"
listas1 = pickle.load(open(file, "rb"))

print(listas[4].list_of_parameters == listas1[4].list_of_parameters)
print(listas[4].list_of_rewards == listas1[4].list_of_rewards)

'''
n_experimentos = len(listas)

for i in range(n_experimentos):
    n_iteraciones = len(listas[i].list_of_rewards)
    list_of_parameters = np.zeros((n_experimentos, n_iteraciones, 5))
    for j in range(n_iteraciones):
        parameters = np.array([listas[i].list_of_parameters[5*j],
                              listas[i].list_of_parameters[(5*j+1)],
                              listas[i].list_of_parameters[(5*j+2)],
                              listas[i].list_of_parameters[(5*j+3)],
                              listas[i].list_of_parameters[(5*j+4)]])
        print(parameters)
        list_of_parameters[i, j] = parameters
    print(len(list_of_parameters[i]))
    listas[i].list_of_parameters = list_of_parameters[i]

pickle.dump(listas, open("listas_difevol_" + TASK_NAME + "_" + VARIATION + "_PRUEBA.p", "wb"))

'''

