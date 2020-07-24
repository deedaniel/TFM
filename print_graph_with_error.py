import pickle
import numpy as np
import matplotlib.pyplot as plt
import statistics as st
import math

file = "listas_bayesopt_waypoints.p"
listas = pickle.load(open(file, "rb"))

n_iteraciones = len(listas[0].iterations)
n_experimentos = len(listas)

best_reward = 1000
lists_of_best_rewards = np.zeros((n_iteraciones, n_experimentos))
list_of_mean_reward = []
list_of_intervals = []
list_of_iterations = listas[0].iterations

for i in range(n_iteraciones):
    for j in range(n_experimentos):
        if i == 0:
            lists_of_best_rewards[i, j] = listas[j].list_of_rewards[i]
        elif listas[j].list_of_rewards[i] < listas[j].list_of_rewards[i-1]:
            lists_of_best_rewards[i, j] = listas[j].list_of_rewards[i]
        else:
            lists_of_best_rewards[i, j] = listas[j].list_of_rewards[i-1]

    mean_reward = st.mean(lists_of_best_rewards[i])
    stddev = st.pstdev(lists_of_best_rewards[i])
    t_student = 2.571
    interval = t_student * stddev / math.sqrt(n_experimentos)

    list_of_mean_reward.append(mean_reward)
    list_of_intervals.append(interval)

figure, ax = plt.subplots()
ax.errorbar(list_of_iterations, list_of_mean_reward, yerr=list_of_intervals)
ax.set(xlabel='iteration', ylabel='best reward', title='Reward vs iterations')
ax.grid()
figure.savefig("it_rew_bayesopt.png")
plt.show()
