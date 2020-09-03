import pickle
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as sp
import math

file = "listas_bayesopt_waypoints_esfericas.p"
listas = pickle.load(open(file, "rb"))

n_iteraciones = len(listas[0].list_of_rewards)
n_experimentos = len(listas)

best_reward = 1000
lists_of_best_rewards = np.zeros((n_experimentos, n_iteraciones))
list_of_mean_reward = []
list_of_intervals = []
list_of_iterations = []

for i in range(n_experimentos):
    for j in range(n_iteraciones):
        lists_of_best_rewards[i, j] = np.min(listas[i].list_of_rewards[:(j + 1)])

res = np.asarray(lists_of_best_rewards)
res_mean = np.mean(lists_of_best_rewards, axis=0)
res_std = np.std(lists_of_best_rewards, axis=0)
n, it = range(res.shape[0]), range(res.shape[1])
t_limits = sp.t.interval(0.95, n_experimentos) / np.sqrt(n_experimentos)

plt.plot(it, res_mean, linewidth=2, label=None)
plt.fill(np.concatenate([it, it[::-1]]),
         np.concatenate([res_mean + t_limits[0] * res_std,
                         (res_mean + t_limits[1] * res_std)[::-1]]),
         alpha=.3)
plt.ylabel(ylabel='Recompensa')
plt.xlabel(xlabel='Iteraciones')
plt.title(label='Recompensa frente iteraciones')
plt.savefig("it_rew_bayesopt_esfericas.png")
plt.show()
