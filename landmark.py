import sys
from pymunk import Body, Circle, ShapeFilter
from configsingleton import ConfigSingleton
from common import *

# Our mechanism for selectively importing pyglet/GUI-related stuff.
import gui_setting
if gui_setting.use:
    from common.drawing import draw_circle

class Landmark(object):
    def __init__(self, mask):
        self.body = Body(0, 0, Body.STATIC)
        self.body.position = 0, 0
        self.body.angle = 0
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0

        #self.radius = 9 * CM_TO_PIXELS
        self.radius = 6 * CM_TO_PIXELS

        self.shape = Circle(self.body, self.radius)

        self.mask = mask
        self.shape.filter = ShapeFilter(mask=ShapeFilter.ALL_MASKS ^ (ANY_PUCK_MASK | ROBOT_MASK), categories = mask)
        if mask == ARC_LANDMARK_MASK:
            self.shape.color = 255, 0, 255
        elif mask == ENTER_ARC_LANDMARK_MASK:
            self.shape.color = 0, 255, 0
        elif mask == EXIT_ARC_LANDMARK_MASK:
            self.shape.color = 255, 0, 0 
        elif mask == POLE_LANDMARK_MASK:
            self.shape.color = 0, 0, 255 
        elif mask == BLAST_LANDMARK_MASK:
            self.shape.color = 255, 255, 255
        else:
            sys.exit("Unknown landmark mask: " + str(mask))

        # The following is just to set the appropriate params to visualize below
        #config = ConfigSingleton.get_instance()
        #self.vis_range_max =  \
        #    config.getfloat("RangeScan:landmarks", "range_max") \
        #    + self.radius
        #self.vis_inside_radius = \
        #    config.getfloat("OutlierBumpController", "inside_radius") \
        #    + self.radius
        #self.vis_outside_radius = \
        #    config.getfloat("OutlierBumpController", "outside_radius") \
        #    + self.radius

    def visualize_params(self):
        pass
        #centre = (self.body.position.x, self.body.position.y)
        #draw_circle(centre, self.vis_range_max, (255, 255, 255))
        #if self.mask == ARC_LANDMARK_MASK:
        #    draw_circle(centre, self.vis_inside_radius, (0, 255, 0))
        #    draw_circle(centre, self.vis_outside_radius, (255, 0, 0))
