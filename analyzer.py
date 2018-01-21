#!/usr/bin/env python

from math import sqrt
from configsingleton import ConfigSingleton
from controllers.distance_point_segment import *

class Analyzer(object):
    def __init__(self, output_dir, landmarks):

        config = ConfigSingleton.get_instance()
        self.lmark_pair_dist = config.get("AlvinSim", "lmark_pair_dist")

        # Open data files and place a comment in the first line of each
        self.avg_puck_segment_dist_file = open(
                '{}/avg_puck_segment_dist.dat'.format(output_dir), 'wa')
        self.avg_puck_segment_dist_file.write('# STEP, AVG_PUCK_SEGMENT_DIST\n')

        self.pucks_file = open('{}/pucks.dat'.format(output_dir), 'wa')
        self.pucks_file.write('# STEP, (X, Y) COORDINATES FOR ALL PUCKS\n')

        self.robots_file = open('{}/robots.dat'.format(output_dir), 'wa')
        self.robots_file.write(
                        '# STEP, (X, Y, THETA) COORDINATES FOR ALL ROBOTS\n')

        # For the landmarks file, we will go ahead and enter it here.
        if len(landmarks) > 0:
            landmarks_file = open('{}/landmarks.dat'.format(output_dir), 'wa')
            landmarks_file.write('# (X, Y) COORDINATES FOR ALL LANDMARKS\n')
            for i in range(len(landmarks)-1):
                pos = landmarks[i].body.position
                landmarks_file.write('{:.5g}, {:.5g}, '.format(pos.x, pos.y))
            pos = landmarks[-1].body.position
            landmarks_file.write('{:.5g}, {:.5g}\n'.format(pos.x, pos.y))

        # Get a list of all landmark pairs for the inter-landmark distance is
        # less than the threshold (meaning that the line segment between them
        # should be filled in).  The result will be list of tuples representing
        # the coordinates of the landmark pairs in the robot reference frame.
        self.lmark_pairs = []
        n = len(landmarks)
        for i in range(n):
            lmark_i  = landmarks[i]
            for j in range(i+1, n):
                lmark_j  = landmarks[j]
                dx = lmark_i.body.position.x - lmark_j.body.position.x
                dy = lmark_i.body.position.y - lmark_j.body.position.y
                inter_lmark_dist = sqrt(dx*dx + dy*dy)
                if inter_lmark_dist < self.lmark_pair_dist:
                    self.lmark_pairs.append((lmark_i, lmark_j))

    def compute_avg_puck_segment_dist(self, steps, pucks):
        """ Compute the average of all puck-to-segment distances, where for
        each puck we find the closest distance to a valid pair of landmarks
        indicating a line segment. """

        sum_puck_segment_dist = 0
        for puck in pucks:
            # Find the closest landmark pair to each puck.
            closest_pair_d = float('inf')
            closest_pair = None
            for pair in self.lmark_pairs:
                d = distance_point_segment(puck.body.position,
                            pair[0].body.position, pair[1].body.position, False)
                if d < closest_pair_d:
                    closest_pair_d = d
            sum_puck_segment_dist += closest_pair_d

        avg_puck_segment_dist = sum_puck_segment_dist / len(pucks)
        self.avg_puck_segment_dist_file.write(
            '{}, {}\n'.format(steps, avg_puck_segment_dist))

    def analyze(self, steps, pucks, robots):

        if len(pucks) == 0:
            return;

        # Do some analysis
        self.compute_avg_puck_segment_dist(steps, pucks)

        # Store the puck and robot positions.
        self.pucks_file.write('{}, '.format(steps))
        for i in range(len(pucks)-1):
            pos = pucks[i].body.position
            self.pucks_file.write('{:.5g}, {:.5g}, '.format(pos.x, pos.y))
        pos = pucks[-1].body.position
        self.pucks_file.write('{:.5g}, {:.5g}\n'.format(pos.x, pos.y))

        self.robots_file.write('{}, '.format(steps))
        for i in range(len(robots)-1):
            body = robots[i].body
            x = body.position.x
            y = body.position.y
            a = body.angle
            self.robots_file.write('{:.5g}, {:.5g}, {:.5g}, '.format(x, y, a))
        body = robots[-1].body
        x = body.position.x
        y = body.position.y
        a = body.angle
        self.robots_file.write('{:.5g}, {:.5g}, {:.5g}\n'.format(x, y, a))
