import pickle
import numpy as np
import param_function as pf
import matplotlib.pyplot as plt

listas = pickle.load(open("save_listas.p", "rb"))

best_reward = 10000
list_of_best_reward = []
for i in range(len(listas.iterations)):
    if listas.list_of_rewards[i] < best_reward:
        best_reward = listas.list_of_rewards[i]
    list_of_best_reward = np.append(list_of_best_reward, best_reward)

figure, ax = plt.subplots()
ax.plot(listas.iterations, list_of_best_reward)
ax.set(xlabel='iterations', ylabel='best reward', title='Reward vs iterations')
ax.grid()
figure.savefig("plot2.png")
plt.show()
