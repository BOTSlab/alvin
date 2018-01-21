#!/usr/bin/env python

import sys, os, shutil, math, time
from math import pi
from time import time
from colorama import init, Fore, Style
from configsingleton import ConfigSingleton

def execute(cmd):
    # Print the command to execute in green
    print(Fore.GREEN + cmd)
    print(Style.RESET_ALL)

    os.system(cmd)

#config = ConfigSingleton.get_instance('default.cfg')
config = ConfigSingleton.get_instance('pair.cfg')

# CUSTOMIZE ONLY THIS BLOCK

config_section = "PointSampleCam"
var_name = "max_distance"
var_list = [0, 50, 100, 1000]
start_trial = 0
stop_trial = 0

"""
config_section = "BumpController"
var_name = "puck_dist_threshold"
var_list = [0, 10, 20, 30, 40, 50, 60]
start_trial = 0
stop_trial = 11
"""

"""
config_section = "AlvinSim"
var_name = "number_robots"
var_list = [1, 5, 10, 20, 30, 40, 50]
start_trial = 0
stop_trial = 29
"""

"""
config_section = "BumpController"
var_name = "use_tracking"
var_list = [False, True]
start_trial = 0
stop_trial = 9
"""

"""
config_section = "BumpController"
var_name = "angular_speed"
var_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
start_trial = 0
stop_trial = 9
"""

# Get the number of trials to be executed in advance.
total_trials = 0
#shape_list = ["C_SHAPE", "T_SHAPE"]
shape_list = ["PAIR"]
for var in var_list: 
    for trial in range(start_trial, stop_trial+1):
        total_trials += 1
print "total trials to execute: {}".format(total_trials)

trial_count = 0
for shape_name in shape_list:
    config.set("AlvinSim", "canned_landmarks_name", shape_name)
    #output_dir = '/users/cs/faculty/av/DATA/AAMAS18/' + shape_name + "/" + var_name
    output_dir = '/Users/av/DATA/PAIR/' + shape_name + "/" + var_name
    #shutil.rmtree(output_dir, ignore_errors=True)
    try:
        os.makedirs(output_dir)
    except OSError:
        print "{} already exists".format(output_dir)

    for var in var_list: 
        config.set(config_section, var_name, var)

        filename_base = "{}/{}".format(output_dir, var)
        filename = filename_base.replace('.', 'p') + '.cfg'

        config.write(open(filename, 'w'))

        for trial in range(start_trial, stop_trial+1):
            # Execute command and keep track of the elapsed time.
            start_time = time()
            execute("./alvinsim.py {} {}".format(filename, trial))
            elapsed_time = time() - start_time
            trial_count += 1

            print "trial time (secs): {}".format(elapsed_time)
            remaining_time = (total_trials - trial_count) * elapsed_time
            print "estimated remaining time (mins): {}".format(remaining_time/60)

