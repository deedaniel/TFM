import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as sp


def plot_optim(results,color,symbol='',label=None):
    
    res = np.asarray(results)
    res_mean = np.mean(res,axis=0)
    res_std = np.std(res,axis=0)
    # print(np.amin(res[:, -1]), last_points[np.argmin(res[:,-1])], ftype)

    it, n = range(res.shape[0]), range(res.shape[1])
    t_limits = sp.t.interval(0.95, n) / np.sqrt(n)

    plt.plot(it, res_mean, linewidth=2, label=label)
    plt.fill(np.concatenate([it, it[::-1]]),
             np.concatenate([res_mean + t_limits[0] * res_std,
                             (res_mean + t_limits[1] * res_std)[::-1]]),
             alpha=.3, fc=color, ec='None')


plt.savefig("it_rew_bayesopt.png")
plt.show()
