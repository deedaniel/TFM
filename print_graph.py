import pickle
import numpy as np
import param_function as pf
import matplotlib.pyplot as plt

file = "listas_bayesopt_tray_circular.p"
listas = pickle.load(open(file, "rb"))

best_reward = 10000
list_of_best_reward = []
for i in range(len(listas.iterations)):
    if listas.list_of_rewards[i] < best_reward:
        best_reward = listas.list_of_rewards[i]
    list_of_best_reward = np.append(list_of_best_reward, best_reward)

figure, ax = plt.subplots()
ax.plot(listas.iterations, list_of_best_reward)
ax.set(xlabel='iteration', ylabel='best reward', title='Reward vs iterations')
ax.grid()
figure.savefig("it_rew_tray_circular.png")
plt.show()
