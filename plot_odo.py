#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

def plot(csv_filename, subplot, title):
    with open(csv_filename) as f:
        data = f.read()

    data = data.split('\n')
    del data[-1] # Remove last empty line

    x = [row.split(',')[0] for row in data]
    y = [row.split(',')[1] for row in data]

    ax = fig.add_subplot(subplot)

    ax.set_title(title)
    ax.set_xlim(0, 200)
    ax.set_ylim(-150, 150)
    ax.axis('equal')

    # ax.plot(x,y, 'r.', label='the data')
    ax.plot(x,y, 'r.')

    # leg = ax.legend()

fig = plt.figure()

plot('odo_alpha_0p01.csv', 141, 'alpha = 0.01')
plot('odo_alpha_0p1.csv', 142, 'alpha = 0.1')
plot('odo_alpha_1p0.csv', 143, 'alpha = 1.0')
plot('odo_alpha_10p0.csv', 144, 'alpha = 10.0')

#plt.set_ylabel('your y label...')

plt.show()
