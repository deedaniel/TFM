import pickle
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as sp

file1 = "listas_bayesopt_slideblock.p"
resultados1 = pickle.load(open(file1, "rb"))

file2 = "listas_scipy_slideblock.p"
resultados2 = pickle.load(open(file2, "rb"))

lista_de_resultados = [resultados1, resultados2]
color = ['b', 'r']
etiqueta = ['bayesopt', 'differential evolution']

for resultado in lista_de_resultados:
    n_iteraciones = len(resultado[0].list_of_rewards)
    n_experimentos = len(resultado)

    lists_of_best_rewards = np.zeros((n_experimentos, n_iteraciones))

    for i in range(n_experimentos):
        for j in range(n_iteraciones):
            lists_of_best_rewards[i, j] = np.max(resultado[i].list_of_rewards[:(j + 1)])

    res = np.asarray(lists_of_best_rewards)
    res_mean = np.mean(lists_of_best_rewards, axis=0)
    res_std = np.std(lists_of_best_rewards, axis=0)
    n, it = range(res.shape[0]), range(res.shape[1])
    t_limits = sp.t.interval(0.95, n_experimentos) / np.sqrt(n_experimentos)

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
plt.savefig("it_rew_waypoints_esfericas.png")
plt.show()