import bayesopt
import numpy as np
from scipy.optimize import rosen

params = {'n_iterations': 20, 'n_iter_relearn': 5, 'n_init_samples': 2}

n = 5  # n dimensions
lb = np.zeros((n,))
ub = 2*np.ones((n,))

mvalue, x_out, error = bayesopt.optimize(rosen, n, lb, ub, params)

print("Result", mvalue, "at", x_out)
