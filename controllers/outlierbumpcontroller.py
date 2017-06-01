""" Forms patterns with respect to landmarks by bumping outlier pucks towards/away from landmarks.  Outliers are defined as those pucks within the field of view which are furthest from their nearest landmark. """

from controller import Controller
from math import fabs, cos, sin, pi, sqrt
from random import random, normalvariate
from common import *
from common.drawing import draw_line
from common.angles import get_smallest_angular_difference, normalize_angle_0_2pi
from configsingleton import ConfigSingleton

class OutlierBumpController(Controller):

    def __init__(self, this_robot, puck_mask):
        """
        puck_mask -- The mask for pucks this controller recognizes
        """
        self.puck_mask = puck_mask

        config = ConfigSingleton.get_instance()

        self.linear_speed = config.getfloat("OutlierBumpController", 
                                            "linear_speed")
        self.angular_speed = config.getfloat("OutlierBumpController",
                                             "angular_speed")
        self.slow_factor = config.getfloat("OutlierBumpController",
                                           "slow_factor")
        self.egress_angle = config.getfloat("OutlierBumpController",
                                             "egress_angle")
        self.inside_radius = config.getfloat("OutlierBumpController",
                                             "inside_radius")
        self.outside_radius = config.getfloat("OutlierBumpController",
                                              "outside_radius")
        self.wander_prob = config.getfloat("OutlierBumpController",
                                           "wander_prob")
        self.wander_mean_duration = config.getfloat("OutlierBumpController",
                                                    "wander_mean_duration")
        self.wander_sigma_duration = config.getfloat("OutlierBumpController",
                                                     "wander_sigma_duration")

        self.this_robot = this_robot

        # Encodes the current behaviour as a string.
        self.behaviour = None

    def set_robot_color(self):
        if self.behaviour == "WANDER":
            self.this_robot.shape.color = 255, 255, 255
        elif self.behaviour == "ARC_INSIDE":
            self.this_robot.shape.color = 255, 0, 255
        elif self.behaviour == "ARC_OUTSIDE":
            self.this_robot.shape.color = 0, 0, 255
        elif self.behaviour == "ENTER_ARC_INSIDE":
            self.this_robot.shape.color = 0, 255, 0
        elif self.behaviour == "ENTER_ARC_OUTSIDE":
            self.this_robot.shape.color = 127, 255, 127
        elif self.behaviour == "EXIT_ARC_INSIDE":
            self.this_robot.shape.color = 255, 0, 0
        elif self.behaviour == "EXIT_ARC_OUTSIDE":
            self.this_robot.shape.color = 255, 127, 127

    def react(self, this_robot, sensor_suite, visualize=False):

        # Store some aspects of the sensor scan so that we don't have to pass
        # them around between functions.
        self.this_robot = this_robot
        self.visualize = visualize
        self.pscan = sensor_suite.range_scan
        self.lscan = sensor_suite.landmark_scan
        self.np = len(self.pscan.ranges)
        self.nl = len(self.lscan.ranges)

        # We will base the state on the type of the closest landmark and its
        # distance.  We may also need the angle of the closest landmark if
        # there are no suitably positioned pucks nearby to bump.
        self.closest_lmark_distance = float('inf')
        self.closest_lmark_angle = None
        self.closest_lmark_mask = None
        for i in range(self.nl):
            if (self.lscan.masks[i] & ANY_LANDMARK_MASK != 0
               and self.lscan.ranges[i] < self.closest_lmark_distance):
                self.closest_lmark_distance = self.lscan.ranges[i]
                self.closest_lmark_angle = self.lscan.angles[i]
                self.closest_lmark_mask = self.lscan.masks[i]

        # Set the current behaviour based on the type of landmark and whether
        # we are inside or outside of its radius.  Wander behaviour is really
        # a separate state and is entered or exited despite the landmark type.
        #
        if self.behaviour == "WANDER":
            self.instate_count -= 1
            if self.instate_count <= 0:
                self.behaviour = "NOT WANDER"
        elif random() < self.wander_prob:
            self.behaviour = "WANDER"
            self.instate_count = normalvariate(self.wander_mean_duration, 
                                               self.wander_sigma_duration)

        if self.behaviour != "WANDER":
            if self.closest_lmark_mask == ARC_LANDMARK_MASK:
                if self.closest_lmark_distance < self.outside_radius:
                    self.behaviour = "ARC_INSIDE"
                else:
                    self.behaviour = "ARC_OUTSIDE"
            elif self.closest_lmark_mask == ENTER_ARC_LANDMARK_MASK:
                if self.closest_lmark_distance < self.outside_radius:
                    self.behaviour = "ENTER_ARC_INSIDE"
                else:
                    self.behaviour = "ENTER_ARC_OUTSIDE"
            elif self.closest_lmark_mask == EXIT_ARC_LANDMARK_MASK:
                if self.closest_lmark_distance < self.outside_radius:
                    self.behaviour = "EXIT_ARC_INSIDE"
                else:
                    self.behaviour = "EXIT_ARC_OUTSIDE"

        self.set_robot_color()

        # Handle behaviour-specific behaviour
        if self.behaviour == "WANDER":
            return self.wander_react()
        elif self.behaviour == "EXIT_ARC_INSIDE":
            return self.curve_away_from(self.closest_lmark_angle)
        elif self.behaviour == "ARC_INSIDE":
            return self.innie_react()
        elif self.behaviour == "ARC_OUTSIDE" \
             or self.behaviour.startswith("ENTER_ARC") \
             or self.behaviour.startswith("EXIT_ARC"):
            return self.outie_react()
#        elif self.behaviour == "ENTER_ARC_INSIDE":
#            return self.curve_to_angle(self.closest_lmark_angle, pi/2)
#        elif self.behaviour == "ENTER_ARC_OUTSIDE":
#            return self.curve_to_angle(self.closest_lmark_angle, 0)
#        elif self.behaviour == "EXIT_ARC_OUTSIDE":
#            return self.curve_to_angle(
#                    normalize_angle_0_2pi(self.closest_lmark_angle), 3*pi/2)

    def outie_react(self):

        # Go through all visible puck pixels and determine for each the closest
        # landmark pixel (MIGHT GET EXPENSIVE!).  Then determine the
        # puck-landmark distance and keep track of the largest such distance
        # and angle.
        largest_closest_dist = 0 # Confusingly named, but accurate 
        largest_closest_angle = None
        for p in range(self.np):
            if self.pscan.masks[p] & self.puck_mask != 0:
                pr = self.pscan.ranges[p]
                pt = self.pscan.angles[p]
                closest_lmark_dist = float('inf')
                closest_lmark_angle = None
                for l in range(self.nl):
                    # Landmark mask must match that landmark closest to robot
                    if self.lscan.masks[l] & self.closest_lmark_mask != 0:
                        lr = self.lscan.ranges[l]
                        lt = self.lscan.angles[l]
                        puck_lmark_dist = sqrt(pr**2 + lr**2 - 2*pr*lr*
                            cos(get_smallest_angular_difference(pt, lt)))
                        if puck_lmark_dist < closest_lmark_dist:
                            closest_lmark_dist = puck_lmark_dist
                            closest_lmark_angle = lt
                if (pt > closest_lmark_angle 
                    and (pt - closest_lmark_angle) < 3*pi/4
                    and closest_lmark_dist > largest_closest_dist):
                    largest_closest_dist = closest_lmark_dist
                    largest_closest_angle = pt
                        
        # The robot should not further disturb pucks that are within their
        # landmark's outer radius.
        if (self.behaviour == "ARC_OUTSIDE"
            and largest_closest_angle != None
            and largest_closest_dist < self.outside_radius):
            largest_closest_angle += self.egress_angle
            if self.visualize:
                draw_line(self.this_robot, self.pscan, largest_closest_angle,
                          (255, 255, 255),width=20)

        return self.generate_twist(largest_closest_angle, True)

    def innie_react(self):
        # Similar to the big loop in 'outie_react' but now we are looking for
        # those pucks which are closest to their closest landmark.
        closest_closest_dist = float('inf')
        closest_closest_angle = None
        for p in range(self.np):
            if self.pscan.masks[p] & self.puck_mask != 0:
                pr = self.pscan.ranges[p]
                pt = self.pscan.angles[p]
                closest_lmark_dist = float('inf')
                closest_lmark_angle = None
                for l in range(self.nl):
                    if self.lscan.masks[l] & self.closest_lmark_mask != 0:
                        lr = self.lscan.ranges[l]
                        lt = self.lscan.angles[l]
                        puck_lmark_dist = sqrt(pr**2 + lr**2 - 2*pr*lr*
                            cos(get_smallest_angular_difference(pt, lt)))
                        if puck_lmark_dist < closest_lmark_dist:
                            closest_lmark_dist = puck_lmark_dist
                            closest_lmark_angle = lt
                if (pt < closest_lmark_angle 
                    and (closest_lmark_angle - pt) < 3*pi/4
                    and closest_lmark_dist < closest_closest_dist):
                    closest_closest_dist = closest_lmark_dist
                    closest_closest_angle = pt
                        
        # The robot should not further disturb pucks that are outside the inner
        # radius.
        if (self.behaviour == "ARC_INSIDE"
            and closest_closest_angle != None
            and closest_closest_dist > self.inside_radius):
            closest_closest_angle -= self.egress_angle
            if self.visualize:
                draw_line(self.this_robot, self.pscan, closest_closest_angle,
                          (255, 255, 255),width=20)

        return self.generate_twist(closest_closest_angle, False)

    def generate_twist(self, target_angle, outie):

        # Check for the presence of another robot in the centre of the image.
        react_to_robot = False
        react_to_robot_angle = None
        self.np = len(self.pscan.ranges)
        for i in range(0, self.np):
            if (self.pscan.masks[i] == ROBOT_MASK and fabs(self.pscan.angles[i]) < pi/4
                and self.pscan.ranges[i] < 2*self.this_robot.radius):
                react_to_robot = True
                react_to_robot_angle = self.index_to_angle(self.pscan, i)

        twist = Twist()
        color = None # For visualization
        if react_to_robot:
            # Turn slow in the open direction
            twist.linear = self.slow_factor * self.linear_speed
            if outie:
                twist.angular = self.angular_speed
            else:
                twist.angular = - self.angular_speed
            color = (255, 0, 0)

        elif (target_angle == None):
            # No suitable target angle.
            if self.behaviour == "ARC_OUTSIDE":
                return self.curve_to_angle(self.closest_lmark_angle, -pi/2)
            elif self.behaviour == "ENTER_ARC_INSIDE":
                return self.curve_to_angle(self.closest_lmark_angle, pi/2)
            elif self.behaviour == "ENTER_ARC_OUTSIDE":
                return self.curve_to_angle(self.closest_lmark_angle, 0)
            elif self.behaviour == "ARC_INSIDE":
                return self.curve_to_angle(self.closest_lmark_angle, pi/2)
            elif self.behaviour == "EXIT_ARC_INSIDE":
                return self.curve_away_from(self.closest_lmark_angle)
            elif self.behaviour == "EXIT_ARC_OUTSIDE":
                return self.curve_to_angle(
                        normalize_angle_0_2pi(self.closest_lmark_angle), 3*pi/2)

        elif (target_angle <= 0):
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = - self.angular_speed
            color = (0, 0, 255)

        else:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular = self.angular_speed
            color = (0, 0, 255)

        if self.visualize:
            if target_angle != None:
                draw_line(self.this_robot, self.pscan, target_angle, color)

        return twist

    def curve_to_angle(self, actual_angle, desired_angle):
        twist = Twist()

        if actual_angle < desired_angle:
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = - self.angular_speed
        else:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular = self.angular_speed

        return twist

    def curve_away_from(self, actual_angle):
        """ Like curve_to_angle but for curving away from an object such that
            goes directly behind the robot. """
        twist = Twist()

        if actual_angle > 0:
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = - self.angular_speed
        else:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular = self.angular_speed

        return twist


    def wander_react(self):
        twist = Twist()
        twist.linear = self.linear_speed
        twist.angular = 5 * (random() - 0.5)
        return twist
