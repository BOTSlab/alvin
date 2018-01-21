""" Implementation of Gauci et al's controller which uses the old range scanner sensor. """

from controller import Controller
from math import fabs, pi
from common import *
from configsingleton import ConfigSingleton

# Our mechanism for selectively importing pyglet/GUI-related stuff.
import gui_setting
if gui_setting.use:
    from common.drawing import draw_ray, draw_segment_wrt_robot

class GauciRangeController(Controller):

    def __init__(self, puck_mask):
        self.puck_mask = puck_mask

        config = ConfigSingleton.get_instance()

        self.front_angle_threshold = config.getfloat("GauciController",
                                            "front_angle_threshold")
        self.linear_speed = config.getfloat("GauciController", "linear_speed")
        self.angular_speed = config.getfloat("GauciController", "angular_speed")
        self.slow_factor = config.getfloat("GauciController", "slow_factor")

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        scan = sensor_suite.range_scan
        n = len(scan.ranges)
        centre_angle = 0

        # EXPERIMENT TO ACHIEVE A MINIMUM SIZE AGGREGATE.
        """
        lscan = sensor_suite.landmark_scan
        closest_lmark_distance = float('inf')
        for i in range(len(lscan.ranges)):
            if (lscan.masks[i] & ARC_LANDMARK_MASK != 0
               and lscan.ranges[i] < closest_lmark_distance):
                closest_lmark_distance = lscan.ranges[i]
        if closest_lmark_distance < 200:
            centre_angle = 0.3
        """

        # Relying on only a single forward-pointing ray causes distant pucks
        # to be easily missed (they may be detected, but too sporadically to
        # attract the robot).  We accept as forward-pointing any sensor ray
        # within 'front_angle_threshold' of zero.  Correspondingly set the
        # two predicates 'react_to_puck' and 'react_to_robot'.
        react_to_puck = False
        react_to_robot = False
        for i in range(n):
            if fabs(scan.angles[i] + centre_angle) <self.front_angle_threshold:
                if (scan.masks[i] & self.puck_mask) != 0:
                    react_to_puck = True
                if scan.masks[i] == ROBOT_MASK:
                    react_to_robot = True

        if react_to_robot:
            react_to_puck = False

        # Now react...
        if react_to_puck:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular =  self.angular_speed
            if visualize:
                draw_ray(this_robot, 0, (255, 0, 255))

        elif react_to_robot:
            # Turn left and slow
            twist.linear = self.linear_speed * self.slow_factor
            twist.angular = self.angular_speed

            if visualize: # Reacting to robot
                draw_ray(this_robot, 0, (255, 0, 0))
        else:
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = -self.angular_speed

            if visualize:
                draw_ray(this_robot, 0, (0, 255, 0))
                
        return twist
