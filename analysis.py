#!/usr/bin/env python

import pyglet
import matplotlib.pyplot as plt
from common.drawing import *

def init(output_dir):
    global all_steps, all_sec_moments, output_file
    all_steps = []
    all_sec_moments = []
    plt.ion()
    plt.title("Second Moment")

    output_file = open('{}/sec_moment.dat'.format(output_dir), 'wa')

def get_sec_moment(pucks):
    # Find the centroid of all pucks (not including immobile pucks).
    cx, cy = 0, 0
    n = 0
    for puck in pucks:
        if not puck.immobile:
            n += 1
            cx += puck.body.position.x
            cy += puck.body.position.y
    if n > 0:
        cx /= float(n)
        cy /= float(n)

    # Show the centroid
    #draw_square((cx, cy), 10, (255, 255, 255), 1)
    
    # Calculate second moment of all pucks, scaled by (4 * radius^2) as per
    # "Clustering Objects with Robots that Do Not Compute":
    sec_moment = 0
    for puck in pucks:
        if not puck.immobile:
            dx = puck.body.position.x - cx
            dy = puck.body.position.y - cy
            sec_moment += dx*dx + dy*dy
    if n > 0:
        radius = pucks[0].radius
        sec_moment /= 4.0 * radius * radius

    return sec_moment

def analyze_puck_distribution(steps, pucks):

    sec_moment = get_sec_moment(pucks)

    output_file.write(str(sec_moment) + '\n')

    # Plot the second moment.
    all_steps.append(steps)
    all_sec_moments.append(sec_moment)
    plt.plot(all_steps, all_sec_moments, 'k-')
    #plt.scatter(steps, sec_moment)
    #plt.pause(0.05)

def save_plots(output_dir):
    plt.savefig('{}/sec_moment.png'.format(output_dir))
