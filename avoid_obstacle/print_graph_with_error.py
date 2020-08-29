import pickle
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as sp
import math

file = "listas_bayesopt_waypoints.p"
listas = pickle.load(open(file, "rb"))

n_iteraciones = len(listas[0].list_of_rewards)
n_experimentos = len(listas)

best_reward = 1000
lists_of_best_rewards = np.zeros((n_experimentos, n_iteraciones))
list_of_mean_reward = []
list_of_intervals = []
list_of_iterations = []

for i in range(n_iteraciones):
    for j in range(n_experimentos):
        if i == 0:
            lists_of_best_rewards[j, i] = listas[j].list_of_rewards[i]
        elif listas[j].list_of_rewards[i] < listas[j].list_of_rewards[i - 1]:
            lists_of_best_rewards[j, i] = listas[j].list_of_rewards[i]
        else:
            lists_of_best_rewards[j, i] = listas[j].list_of_rewards[i - 1]

print(lists_of_best_rewards.shape)

res = np.asarray(lists_of_best_rewards)
print(res.shape)
res_mean = np.mean(lists_of_best_rewards, axis=0)
print(res_mean.shape)
res_std = np.std(lists_of_best_rewards, axis=0)
print(res_std.shape)
n, it = range(res.shape[0]), range(res.shape[1])
t_limits = sp.t.interval(0.95, n_experimentos) / np.sqrt(n_experimentos)
print(t_limits.shape)

plt.plot(it, res_mean, linewidth=2, label=None)
plt.fill(np.concatenate([it, it[::-1]]),
         np.concatenate([res_mean + t_limits[0] * res_std,
                         (res_mean + t_limits[1] * res_std)[::-1]]))

plt.savefig("it_rew_bayesopt.png")
plt.show()
