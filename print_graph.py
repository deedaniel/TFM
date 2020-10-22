import pickle
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as sp

TASK_DIR = "push_button/"
TASK_NAME = "push_button"
# coords_type = "esfericas"

file1 = TASK_DIR + "listas_bayesopt_" + TASK_NAME + ".p"
resultados1 = pickle.load(open(file1, "rb"))

# file2 = TASK_DIR + "listas_difevol_" + TASK_NAME + ".p"
# resultados2 = pickle.load(open(file2, "rb"))

# lista_de_resultados = [resultados1, resultados2]
lista_de_resultados = [resultados1]
color = ['b', 'r']
etiqueta = ['bayesopt', 'differential evolution']

for resultado in lista_de_resultados:
    n_iteraciones = 0
    n_experimentos = len(resultado)

    # Las iteraciones se calcula de esta forma porque en la optimización con evolucion diferencial el numero de
    # iteraciones no es fijo
    for i in range(n_experimentos):
        if len(resultado[i].list_of_rewards) > n_iteraciones:
            n_iteraciones = len(resultado[i].list_of_rewards)

    lists_of_best_rewards = np.zeros((n_experimentos, n_iteraciones))

    for i in range(n_experimentos):
        for j in range(n_iteraciones):
            lists_of_best_rewards[i, j] = np.max(resultado[i].list_of_rewards[:(j + 1)])

    print(lists_of_best_rewards[:, n_iteraciones-1])

    res = np.asarray(lists_of_best_rewards)
    res_mean = np.mean(lists_of_best_rewards, axis=0)
    res_std = np.std(lists_of_best_rewards, axis=0)
    n, it = range(res.shape[0]), range(res.shape[1])
    t_limits = sp.t.interval(0.95, n_experimentos) / np.sqrt(n_experimentos)

    # res = res[0:50]
    # res_mean = res_mean[0:50]
    # res_std = res_std[0:50]
    # it = it[0:50]

    plt.plot(it, res_mean, linewidth=2, label=None)
    plt.fill(np.concatenate([it, it[::-1]]),
             np.concatenate([res_mean + t_limits[0] * res_std,
                             (res_mean + t_limits[1] * res_std)[::-1]]),
             alpha=.3, fc=color[lista_de_resultados.index(resultado)],
             label=etiqueta[lista_de_resultados.index(resultado)])

plt.ylabel(ylabel='Recompensa')
plt.xlabel(xlabel='Iteraciones')
plt.title(label='Recompensa frente iteraciones')
plt.legend(loc='lower right')
plt.savefig(TASK_DIR + "it_reward_" + TASK_NAME + ".png")
plt.show()
