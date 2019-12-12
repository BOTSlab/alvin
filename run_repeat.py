#!/usr/bin/env python

import sys, os, shutil, math, time
from math import pi
from time import time
from colorama import init, Fore, Style
from configsingleton import ConfigSingleton

def execute(cmd):
    # Print the command to execute in green
    print((Fore.GREEN + cmd))
    print((Style.RESET_ALL))

    os.system(cmd)

config_filename = 'one_robot.cfg'
config = ConfigSingleton.get_instance(config_filename)

start_trial = 0
stop_trial = 0

#shape_list = ["C_SHAPE", "T_SHAPE"]
shape_list = ["C_SHAPE"]

for shape_name in shape_list:
    config.set("AlvinSim", "canned_landmarks_name", shape_name)
    #output_dir = '/users/cs/faculty/av/DATA/AAMAS18/' + shape_name + "/"
    output_dir = '/Users/av/DATA/AAMAS18/' + shape_name + "/"
    #shutil.rmtree(output_dir, ignore_errors=True)
    try:
        os.makedirs(output_dir)
    except OSError:
        print("{} already exists".format(output_dir))

    filename = output_dir + config_filename

    config.write(open(filename, 'w'))

    for trial in range(start_trial, stop_trial+1):
        # Execute command and keep track of the elapsed time.
        start_time = time()
        execute("./alvinsim.py {} {}".format(filename, trial))
        elapsed_time = time() - start_time

        print("trial time (secs): {}".format(elapsed_time))
        remaining_time = (stop_trial + 1 - trial) * elapsed_time
        print("estimated remaining time (mins): {}".format(remaining_time/60))

