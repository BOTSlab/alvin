""" A RangeScanner emulates a laser scanner and senses walls, other robots, but not pucks. """

import pymunk, pyglet, copy
from math import pi, cos, sin
from pymunk import ShapeFilter
from common import *
from robot import Robot

from configsingleton import ConfigSingleton

class RangeScan:
    """ A scan consists of a predfined list of angles, computed lists of ranges and masks, as well as associated constants. """

    def __init__(self, config_section):

        config = ConfigSingleton.get_instance()
        self.config_section = config_section
        self.NUMBER_POINTS = config.getint(config_section, "number_points")
        self.ANGLE_MIN = config.getfloat(config_section, "angle_min")
        self.ANGLE_MAX = config.getfloat(config_section, "angle_max")
        #self.RANGE_MIN = config.getfloat(config_section, "range_min")
        self.RANGE_MAX = config.getfloat(config_section, "range_max")

        self.angles = []
        if self.NUMBER_POINTS == 1:
            self.angles.append(self.ANGLE_MIN)
        else:
            angle_delta = (self.ANGLE_MAX - self.ANGLE_MIN) / \
                          (self.NUMBER_POINTS - 1)
            for i in range(self.NUMBER_POINTS):
                self.angles.append(self.ANGLE_MIN + i * angle_delta)

        self.ranges = []
        self.masks = []

        #self.INNER_RADIUS = robot.radius + 5
        #self.INNER_RADIUS = robot.radius + self.RANGE_MIN
        #self.OUTER_RADIUS = self.INNER_RADIUS + self.RANGE_MAX
        self.OUTER_RADIUS = self.RANGE_MAX

class RangeScanner:
    def __init__(self, range_scan_sec_name, detection_mask, acceptance_mask):
        # The detection mask is used to indicate all types of objects that
        # the sensor should be sensitive to.  However, if a detected object
        # doesn't also match the acceptance mask then it will be treated as
        # a wall.
        self.detection_mask = detection_mask
        self.acceptance_mask = acceptance_mask

        self.range_scan_sec_name = range_scan_sec_name

        # Upon the first call to 'compute' we will initialize the following
        # array which stores the ranges from the robot's centre at which to
        # start ray-casting.  
        self.start_ranges = None

    def init_start_ranges(self):

        self.start_ranges = []

        # A throwaway space, so that we can place the robot within it and to
        # ray-casts without the presence of any other objects.
        temp_robot = Robot()
        temp_space = pymunk.Space()
        temp_space.add(temp_robot.body, temp_robot.shape)

        # A throwaway scan, just to get the angles to be used.
        temp_scan = RangeScan(self.range_scan_sec_name)

        # For each sensor angle we will cast a ray in from the outer limit of
        # the sensor's range inward, keeping track of the position at which the
        # robot's body is encountered.
        for sensor_angle in temp_scan.angles:
            c = cos(temp_robot.body.angle + sensor_angle)
            s = sin(temp_robot.body.angle + sensor_angle)
            x1 = int(temp_robot.body.position.x + temp_scan.OUTER_RADIUS * c)
            y1 = int(temp_robot.body.position.y + temp_scan.OUTER_RADIUS * s)
            x2 = int(temp_robot.body.position.x)
            y2 = int(temp_robot.body.position.y)

            #mask = ShapeFilter.ALL_MASKS ^ 2
            shape_filter = ShapeFilter(mask=ROBOT_MASK)
            query_info = temp_space.segment_query_first((x1, y1), (x2, y2), 1, \
                                                 shape_filter)
            assert query_info != None
            inward_range = query_info.alpha * temp_scan.OUTER_RADIUS

            self.start_ranges.append(temp_scan.OUTER_RADIUS - inward_range
                                    + 4 )


    def compute(self, env, robot):
        """ Returns a Scan taken from the given environment and robot. """

        if self.start_ranges == None:
            self.init_start_ranges()

        scan = RangeScan(self.range_scan_sec_name)

        for i in range(scan.NUMBER_POINTS):
            sensor_angle = scan.angles[i]
            c = cos(robot.body.angle + sensor_angle)
            s = sin(robot.body.angle + sensor_angle)
            start_range = self.start_ranges[i]
            x1 = int(robot.body.position.x + start_range * c)
            y1 = int(robot.body.position.y + start_range * s)
            x2 = int(robot.body.position.x + scan.OUTER_RADIUS * c)
            y2 = int(robot.body.position.y + scan.OUTER_RADIUS * s)

            #mask = ShapeFilter.ALL_MASKS ^ 2
            shape_filter = ShapeFilter(mask=self.detection_mask)
            query_info = env.segment_query_first((x1, y1), (x2, y2), 1, \
                                                 shape_filter)
            if query_info == None or query_info.shape == None:
                scan.ranges.append(scan.RANGE_MAX)
                scan.masks.append(0)

            else:
                range_from_robot_centre = start_range + \
                               query_info.alpha * (scan.RANGE_MAX - start_range)
                object_mask = query_info.shape.filter.categories
                if object_mask & self.acceptance_mask == 0:
                    # The detected shape is not accepted, we will treat
                    # it as a wall.
                    object_mask = WALL_MASK

                scan.ranges.append(range_from_robot_centre)
                scan.masks.append(object_mask)

        return scan

    def visualize(self, robot, scan):
        for i in range(scan.NUMBER_POINTS):
            r = scan.ranges[i]
            start_range = self.start_ranges[i]
            sensor_angle = scan.angles[i]
            object_mask = scan.masks[i]

            c = cos(robot.body.angle + sensor_angle)
            s = sin(robot.body.angle + sensor_angle)
            x1 = int(robot.body.position.x + start_range * c)
            y1 = int(robot.body.position.y + start_range * s)
            x2 = int(robot.body.position.x + r*c)
            y2 = int(robot.body.position.y + r*s)
            if object_mask == WALL_MASK:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ('v2f', (x1, y1, x2, y2)),
                             ('c3B', (255, 255, 0, 255, 255, 0)))
                #pass
            elif object_mask == ROBOT_MASK:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ('v2f', (x1, y1, x2, y2)),
                             ('c3B', (0, 255, 255, 0, 255, 255)))
            elif object_mask == BLAST_LANDMARK_MASK:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ('v2f', (x1, y1, x2, y2)),
                             ('c3B', (255, 100, 100, 255, 100, 100)))
            elif object_mask == POLE_LANDMARK_MASK:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ('v2f', (x1, y1, x2, y2)),
                             ('c3B', (100, 100, 255, 100, 100, 255)))
            elif object_mask == ARC_LANDMARK_MASK:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ('v2f', (x1, y1, x2, y2)),
                             ('c3B', (100, 255, 100, 100, 255, 100)))
            elif object_mask == RED_PUCK_MASK:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ('v2f', (x1, y1, x2, y2)),
                             ('c3B', (255, 0, 0, 255, 0, 0)))
            elif object_mask == GREEN_PUCK_MASK:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ('v2f', (x1, y1, x2, y2)),
                             ('c3B', (0, 255, 0, 0, 255, 0)))
            elif object_mask == BLUE_PUCK_MASK:
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES,
                             ('v2f', (x1, y1, x2, y2)),
                             ('c3B', (0, 0, 255, 0, 0, 255)))
