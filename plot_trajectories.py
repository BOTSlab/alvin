#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from common import CM_TO_PIXELS
from math import cos, sin, pi
from matplotlib.collections import EllipseCollection

def scaled_scatter_plot(ax, X, Y, color, radius, filled=True):
    """ Regular scatter plots require the size of markers to be given in
pixels, not relative to the data.  This does relative-sized markers.  Taken
from
https://stackoverflow.com/questions/33094509/correct-sizing-of-markers-in-scatter-plot-to-a-radius-r-in-matplotlib.
"""
    offsets = list(zip(X, Y))
    size = 2*radius 
    if filled:
        ax.add_collection(EllipseCollection(widths=size, heights=size, angles=0, units='xy', facecolors=color, offsets=offsets, transOffset=ax.transData))
    else:
        ax.add_collection(EllipseCollection(widths=size, heights=size, angles=0, units='xy', edgecolors='k', offsets=offsets, transOffset=ax.transData))

def circular_border(ax):
    offsets = [width/2, height/2]
    color = 'k'
    size = width - 10
    ax.add_collection(EllipseCollection(widths=size, heights=size, angles=0, units='xy', facecolors='w', edgecolors='k', offsets=offsets, transOffset=ax.transData))


def plot(ax, filename, desired_step=None):
    with open(filename) as f:
        data = f.read()

    data = data.split('\n')
    del data[0] # Remove first comment line
    del data[-1] # Remove last empty line

    if 'robots.dat' in filename:
        X = []
        Y = []
        DX = []
        DY = []
        for row in data:
            tokens = row.split(',')
            step = int(tokens[0])
            if step == desired_step:
                for i in range(1, len(tokens), 3):
                    X.append(float(tokens[i]))
                    Y.append(float(tokens[i+1]))
                    theta = float(tokens[i+2])
                    DX.append(cos(theta))
                    DY.append(sin(theta))

        size = 9 * CM_TO_PIXELS
        #plt.scatter(X, Y, color='g', s=size_factor*size)
        scaled_scatter_plot(ax, X, Y, 'g', size)

        plt.quiver(X, Y, DX, DY, scale=15.0)

    elif 'pucks.dat' in filename:
        X = []
        Y = []
        for row in data:
            tokens = row.split(',')
            step = int(tokens[0])
            if step == desired_step:
                for i in range(1, len(tokens), 2):
                    X.append(float(tokens[i]))
                    Y.append(float(tokens[i+1]))

        size = 12 * CM_TO_PIXELS
        #plt.scatter(X, Y, color='r', s=size_factor*size)
        scaled_scatter_plot(ax, X, Y, 'r', size)

    elif 'landmarks.dat' in filename:
        X = []
        Y = []
        for row in data:
            tokens = row.split(',')
            for i in range(0, len(tokens), 2):
                X.append(float(tokens[i]))
                Y.append(float(tokens[i+1]))

        size = 10 * CM_TO_PIXELS
        #plt.scatter(X, Y, color='b', s=size_factor*size)
        scaled_scatter_plot(ax, X, Y, 'b', size, filled=False)

    plt.axis('square')
    plt.xlim(0, width)
    plt.ylim(0, height)
    plt.tick_params(
        axis='both',
        which='both',
        top='off',
        bottom='off',
        left='off',
        right='off',
        labelbottom='off',
        labelleft='off')
    plt.title(desired_step)

def do_plots(direc, step, png_output):
    global save_index
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.axis('off')
    circular_border(ax)
    plot(ax, direc + 'pucks.dat', step)
    plot(ax, direc + 'landmarks.dat')
    plot(ax, direc + 'robots.dat', step)
    if not png_output:
        # PDF output
        filename = '{}/{:07d}.pdf'.format(direc, save_index)
        fig.savefig(filename, bbox_inches='tight')
        plt.show()
    else:
        # PNG output (for making videos) 
        filename = '{}/{:07d}.png'.format(direc, save_index)
        fig.savefig(filename)
        plt.close()
    save_index += 1

global width, height
width = 900
height = 900
save_index = 0

dir_base = '/Users/av/DATA/MRS17/'
dir_name = 'T_SHAPE'
var_name = 'number_robots'
var_value = '5'
n_trials = 5
number_steps = 20000
increment = 20000
png_output = False

for i in range(n_trials):
    direc = '{}/{}/{}/{}/{}/'.format(dir_base, dir_name, var_name, var_value, i)
    for step in range(0, number_steps + 1, increment):
        do_plots(direc, step, png_output)
