from math import log, cos, sqrt, pi
from random import random, gauss
import numpy as np

def internal_rand_normal():
    """Approximate sampling from the standard normal distribution
    -- From http://osa1.net/posts/2012-12-19-different-distributions-from-uniform.html
    """
    return sqrt(-2 * log(random())) * cos(2 * pi * random()) / 2

n = 100000
print("Using internal_rand_normal():")
i_vals = []
for i in range(n):
    v = internal_rand_normal()
    i_vals.append(v)

i_array = np.array(i_vals)
print("mean: {}, std: {}".format(np.mean(i_array), np.std(i_array)))

print("Using gauss(0, 1):")
g_vals = []
for i in range(n):
    g = gauss(0, 1)
    g_vals.append(g)

g_array = np.array(g_vals)
print("mean: {}, std: {}".format(np.mean(g_array), np.std(g_array)))
