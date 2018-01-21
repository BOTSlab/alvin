""" A probe just sits in space and processes sensor data---but unlike a robot,
it does not move. """

from pymunk import Body, moment_for_circle
from math import cos, sin, atan2, pi
from common import *

# Our mechanism for selectively importing pyglet/GUI-related stuff.
import gui_setting
if gui_setting.use:
    from common.drawing import draw_ray, draw_circle

class Probe(object):
    def __init__(self):
        # We'll have a body, just to have a way of representing its position
        # and treating it similarly to a robot (where convenient).
        self.body = Body(0, 0, Body.STATIC)
        self.body.position = 0, 0
        self.body.angle = 0
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0

        self.radius = 0

    def get_lmark_dist_angle_mask(self, lscan):
        """ Return a tuple (distance, angle, mask) of the closest landmark. """
    
        closest_distance = float('inf')
        closest_angle = None
        closest_mask = None
        closest_index = None
        for i in range(len(lscan.ranges)):
            if (lscan.masks[i] & ANY_LANDMARK_MASK != 0
               and lscan.ranges[i] < closest_distance):
                closest_distance = lscan.ranges[i]
                closest_angle = lscan.angles[i]
                closest_mask = lscan.masks[i]
                closest_index = i
        if closest_angle == None:
            return (None, None, None)

        return (closest_distance, closest_angle, closest_mask)

    def react(self, lscan):

        (cl_dist, cl_angle, cl_mask) = self.get_lmark_dist_angle_mask(lscan)

        draw_circle(self.body.position, 2, (255, 255, 255))

        if cl_angle != None and cl_dist > 0:
            if cl_mask == ARC_LANDMARK_MASK:
                draw_ray(self, cl_angle, (0, 255, 0), 20, 1)
            elif cl_mask == BLAST_LANDMARK_MASK:
                draw_ray(self, cl_angle+pi, (255, 0, 0), 20, 1)
            elif (cl_mask == POLE_LANDMARK_MASK):

                # We'll average all pole landmark responses.
                vx = 0
                vy = 0
                for i in range(len(lscan.ranges)):
                    if (lscan.masks[i] & POLE_LANDMARK_MASK != 0):
                        vx += cos(lscan.angles[i])
                        vy += sin(lscan.angles[i])
                combined_angle = atan2(vy, vx)
                
                draw_ray(self, combined_angle, (0, 0, 255), 20, 1)
