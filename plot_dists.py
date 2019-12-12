#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from operator import add

#plt.rcParams['axes.prop_cycle'] = ("cycler('color', 'rgykcm') *"
#                                   "cycler('linestyle', ['-', '--', ':', '-.'])")

def plot(filename):
    with open(filename) as f:
        data = f.read()

    data = data.split('\n')
    del data[0] # Remove first comment line
    del data[-1] # Remove last empty line

    x = [row.split(',')[0] for row in data]
    y = [row.split(',')[1] for row in data]

    plt.plot(x,y)

def plot_average(filename_list, legend):

    steps = None
    sums = None

    # Read through all files and compute statistics on the second column.
    for filename in filename_list:
        with open(filename) as f:
            data = f.read()
        print(filename)

        data = data.split('\n')
        del data[0] # Remove first comment line
        del data[-1] # Remove last empty line

        if sums == None:
            first_col = [row.split(',')[0] for row in data]
            second_col = [row.split(',')[1] for row in data]
            steps = list(map(float, first_col))
            sums = list(map(float, second_col))
        else:
            second_col = [row.split(',')[1] for row in data]
            second_col_float = list(map(float, second_col))
            sums = list(map(add, sums, second_col_float))
            
    n = len(filename_list)
    avgs = [s/n for s in sums]

    #legend = legend.replace('p', '.')
    plt.plot(steps,avgs, label=legend)
    plt.xlabel("Steps")
    plt.ylabel("Average Puck-Segment Distance")


def prepare_plot_average(dir_base, dir_name, var_name, value_list, n_trials):
    f = 'avg_puck_segment_dist.dat'

    if len(value_list) == 0:
        d = '{}/{}/{}'.format(dir_base, dir_name, var_name)
        filename_list = ['{}/{}/{}'.format(d,i,f) for i in range(n_trials)]
        plot_average(filename_list, var_name)
        plt.legend()
        return

    for value in value_list:
        d = '{}/{}/{}/{}'.format(dir_base, dir_name, var_name, value)
        filename_list = ['{}/{}/{}'.format(d,i,f) for i in range(n_trials)]
        plot_average(filename_list, '{} = {}'.format(var_name, value))
        plt.legend()

fig = plt.figure()

"""
dir_base = '/Users/av/DATA'
dir_name = 'C_SHAPE'
plt.title(dir_name)
prepare_plot_average(dir_base, dir_name, 'odo_home_timeout', ['50', '100', '150'], 5)
prepare_plot_average(dir_base, dir_name, 'odo_home_factor', ['0p95', '1p0', '1p05'], 5)
prepare_plot_average(dir_base, dir_name, 'NO_ODO', [], 3)
"""

#dir_base = '/Users/av/DATA/AAMAS18/'
dir_base = '/Users/av/DATA/PAIR/'
#dir_name = 'C_SHAPE'
dir_name = 'PAIR'
#plt.title(dir_name)
#prepare_plot_average(dir_base, dir_name, 'single_condition', ['SIMPLE', 'ON_RIGHT', 'ONLY_IF_NO_PAIR'], 10)
#prepare_plot_average(dir_base, dir_name, 'pair_condition', ['SIMPLE', 'BACK_RIGHT', 'FRONT_POSITIVE', 'BOTH'], 10)
#prepare_plot_average(dir_base, dir_name, 'puck_condition', ['SIMPLE', 'ALPHA', 'BETA', 'BOTH'], 10)
#prepare_plot_average(dir_base, dir_name, 'odo_home_factor', ['0p25', '0p5', '0p75', '0p95', '1p0', '1p05'], 5)
#prepare_plot_average('.', '', '', ['before', 'default'], 1)
#prepare_plot_average('.', 'default', '', [], 1)
#prepare_plot_average(dir_base, dir_name, 'tracking_threshold', [10, 20, 30, 50, 100, 1000000], 5)
#prepare_plot_average(dir_base, dir_name, 'puck_dist_threshold', [0, 10, 20, 30, 40, 50], 12)
#prepare_plot_average(dir_base, dir_name, 'lmark_ideal_range', [0, 50, 150, 200, 300], 12)
#prepare_plot_average(dir_base, dir_name, 'number_robots', [1, 5, 10, 20, 30, 40, 50], 30)
#prepare_plot_average(dir_base, dir_name, 'use_tracking', [False, True], 10)
#prepare_plot_average(dir_base, dir_name, 'angular_speed', [1, 2, 3, 4, 5, 6, 7, 8], 6)
#prepare_plot_average(dir_base, dir_name, 'puck_dist_threshold', [25, 30, 35, 40, 45, 50], 7)
#prepare_plot_average(dir_base, dir_name, 'wall_turn_on_spot', [False, True], 7)
#prepare_plot_average(dir_base, dir_name, 'no_target_turn_on_spot', [False, True], 7)
#prepare_plot_average(dir_base, dir_name, 'outer_exclude_angle', ['1p57079632679', '2p35619449019'], 7)
prepare_plot_average(dir_base, dir_name, 'max_distance', [0, 50, 100, 1000], 1)

plt.show()
#fig.savefig('lmark_ideal_range_C.pdf', bbox_inches='tight')
