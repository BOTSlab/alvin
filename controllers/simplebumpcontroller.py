""" A simplified fork of BumpController.  Assumes there is only one landmark. """

from .controller import Controller
from math import acos, atan2, exp, fabs, cos, sin, pi, sqrt, acos
from numpy import sign
from random import uniform
from common import *
from common.angles import get_smallest_angular_difference, \
                          get_smallest_signed_angular_difference
from configsingleton import ConfigSingleton
from controllers.distance_point_segment import *

# Our mechanism for selectively importing pyglet/GUI-related stuff.
import gui_setting
if gui_setting.use:
    from common.drawing import draw_ray, draw_segment_wrt_robot

class SimpleBumpController(Controller):

    def __init__(self, robot, puck_mask):
        """
        puck_mask -- The mask for pucks this controller recognizes
        """
        self.puck_mask = puck_mask

        config = ConfigSingleton.get_instance()

        self.linear_speed = config.getfloat("SimpleBumpController", 
                                            "linear_speed")
        self.angular_speed = config.getfloat("SimpleBumpController",
                                             "angular_speed")
        self.slow_factor = config.getfloat("SimpleBumpController",
                                           "slow_factor")
        self.puck_dist_threshold = config.getfloat("SimpleBumpController",
                                           "puck_dist_threshold")
        self.robot_react_range = config.getfloat("SimpleBumpController",
                                           "robot_react_range")
        self.wall_react_range = config.getfloat("SimpleBumpController",
                                           "wall_react_range")
        self.inner_exclude_angle = config.getfloat("SimpleBumpController",
                                           "inner_exclude_angle")
        self.outer_exclude_angle = config.getfloat("SimpleBumpController",
                                           "outer_exclude_angle")
        self.lmark_ideal_range = config.getfloat("SimpleBumpController",
                                           "lmark_ideal_range")

        # One of: "SIMPLE", "ALPHA", "BETA", "BOTH"
        self.puck_condition = config.get("SimpleBumpController", "puck_condition")

        self.wall_turn_on_spot = config.getboolean("SimpleBumpController",
                                           "wall_turn_on_spot")
        self.no_target_turn_on_spot = config.getboolean("SimpleBumpController",
                                           "no_target_turn_on_spot")
        self.curve_gamma = config.getfloat("SimpleBumpController",
                                           "curve_gamma")

        self.lmark_pair_dist = config.getint("AlvinSim", "lmark_pair_dist") + 10

        self.robot = robot

        # Position of the target
        self.target_pos = None

    def react(self, robot, sensor_dump, visualize=False):
        self.process_landmarks(robot, sensor_dump, visualize)

        if self.guidance_vec == None:
            self.target_pos = None
        else:
            self.process_pucks(robot, sensor_dump, visualize)

        return self.act(robot, sensor_dump, visualize)

    def process_landmarks(self, robot, sensor_dump, visualize=False):
        """ Process the visible landmarks and set the following var's:

            self.guidance_vec   -- Vector from the robot to the 
                                   closest landmark.
            self.guidance_angle -- Angle of self.guidance_vec.
        """

        lmark_scan = sensor_dump.landmarks
        nLmarks = len(lmark_scan.landmarks)
        self.guidance_vec = None
        self.guidance_angle = None

        closest_d = float('inf')
        for i in range(nLmarks):
            a = lmark_scan.landmarks[i].angle
            r = lmark_scan.landmarks[i].distance

            if (r < closest_d):
                closest_d = r
                self.guidance_vec = (r * cos(a), r * sin(a))
                self.guidance_angle = a


    def process_pucks(self, robot, sensor_dump, visualize=False):
        """ If self.target_pos is set (not None) then process the puck pixels
        in the vicinity of this target.  If it is not set, consider all puck
        pixels.
        """

        image = sensor_dump.lower_image

        # We'll use the vector 'normal' below which is the guidance vector 
        # (vector to the landmark) rotated by pi/2.
        normal = (-self.guidance_vec[1], self.guidance_vec[0])
        normal_length = sqrt(normal[0]**2 + normal[1]**2)

        # Now we look at all puck pixels and calculate the perpendicular dist
        # from each to the landmark.  We set the target to the puck with the 
        # largest d (i.e. the largest-closest distance).
        new_target_selected = False
        largest_d = 0
        largest_dp = None
        for i in range(image.n_rows):
            if image.masks[i] & self.puck_mask != 0:
                (xr, yr) = image.calib_array[i,2], image.calib_array[i,3]
                pt = image.calib_array[i,4]
                pr = image.calib_array[i,5]
                puck_vec_length = sqrt(xr**2 + yr**2)

                d = sqrt(distance_squared((xr, yr), self.guidance_vec))

                dot_product = xr * normal[0] + yr * normal[1]
                norm_dot_product = dot_product / (normal_length * puck_vec_length)
                #angle_between_normal_and_puck = acos(dot_product / (normal_length * puck_vec_length))

                if visualize:
                    draw_segment_wrt_robot(robot, (xr, yr),
                        self.guidance_vec, color=(255,0,255), width=3)
                    
                condition_okay = True
                if (self.puck_condition == "SIMPLE"):
                    pass

                #elif (self.puck_condition == "DOT" and fabs(angle_between_normal_and_puck) < pi/4):
                #    condition_okay = False
                elif (self.puck_condition == "DOT" and norm_dot_product < 0.75):
                    condition_okay = False

                elif (self.puck_condition == "BETA"
                    and (pt - self.guidance_angle) >= self.outer_exclude_angle):
                    condition_okay = False

                elif (self.puck_condition == "BOTH"
                    and ((pt - self.guidance_angle) <= self.inner_exclude_angle)
                    or ((pt - self.guidance_angle) >= self.outer_exclude_angle)):
                    condition_okay = False

                if (condition_okay
                    and d > self.puck_dist_threshold
                    and d > largest_d):
                    largest_d = d
                    #largest_dp = angle_between_normal_and_puck
                    largest_dp = dot_product
                    self.target_pos = (xr, yr)
                    new_target_selected = True
                    #    (pr, pt + self.target_angle_bias)
                
        if new_target_selected:
            print("largest_dp: {}".format(largest_dp), end='')

        if not new_target_selected:
            self.target_pos = None

    def act(self, robot, sensor_dump, visualize=False):
        """ Set wheel speeds according to processed sensor data. """

        image = sensor_dump.lower_image

        # Check if there are no landmarks in view, but there are wall pixels we
        # can use for guidance.
        react_to_wall = False
        react_to_wall_angle = None
        closest_wall_range = self.wall_react_range
        for i in range(image.n_rows):
            if image.masks[i] == WALL_MASK:
                wall_angle = image.calib_array[i,4]
                wall_range = image.calib_array[i,5]
                if wall_range < closest_wall_range:
                    closest_wall_range = wall_range
                    react_to_wall_angle = wall_angle
                    react_to_wall = True

        # Check for the presence of another robot in the centre of the image.
        react_to_robot = False
        react_to_robot_angle = None
        for i in range(image.n_rows):
            if image.masks[i] == ROBOT_MASK:
                (xr, yr) = image.calib_array[i,2], image.calib_array[i,3]
                bot_a = image.calib_array[i,4]
                bot_r = image.calib_array[i,5]

                if fabs(bot_a) < pi/4 and bot_r < self.robot_react_range:
                    react_to_robot = True
                    react_to_robot_angle = bot_a

        # Quantity referenced in a couple of places below:
        if self.guidance_vec != None:
            dist_to_lmarks = sqrt(self.guidance_vec[0]**2 + 
                                  self.guidance_vec[1]**2)

        twist = Twist()
        action_print = None
        if react_to_robot:
            # Turn slow in the open direction
            twist.linear = self.slow_factor * self.linear_speed
            twist.angular = self.angular_speed
            action_print = "ROBOT"
            if visualize:
                draw_ray(robot, react_to_robot_angle, color=(255,0,0))

        elif react_to_wall:
            action_print = "WALL"
            if visualize:
                draw_ray(robot, react_to_wall_angle, color=(255,0,255))
            twist.linear = uniform(-0.1, 0.1) * self.linear_speed
            twist.angular = -self.angular_speed

        elif self.guidance_vec == None:
            # No landmarks in view AND no walls in view.  Just go straight.
            twist.linear = self.linear_speed
            action_print = "STRAIGHT (NO LANDMARKS OR WALLS IN VIEW)"

        elif self.target_pos != None and dist_to_lmarks > self.puck_dist_threshold:
            angle = atan2(self.target_pos[1], self.target_pos[0])
            twist.linear = self.linear_speed * cos(angle)
            twist.angular = self.angular_speed * sin(angle)

            action_print = "TARGETING"
            if visualize:
                draw_segment_wrt_robot(robot, (0, 0), 
                                       (self.target_pos[0], self.target_pos[1]),
                                       color=(255, 255, 255), width=3)
        else:
            dist = dist_to_lmarks
            ideal_dist = self.lmark_ideal_range

            # We want to use proportional control to keep the robot at
            # self.lmark_ideal_range away from the landmarks, while facing
            # orthogonal to them.  So we need an error signal for distance.
            # The most simpleminded such signal is this one:
            #       dist_error = ideal_dist - dist
            # But its unknown scale presents a problem.  So instead we use the 
            # following which lies in the range [-1, 1]:
            dist_error = (ideal_dist - min(dist, 2*ideal_dist)) / (ideal_dist)
            # One consequence of using this error signal is that the response
            # at a distance greater than 2*ideal_dist is the same as at
            # 2*ideal_dist.

            # But we also have to consider the current angle w.r.t. the
            # guidance angle.  Ideally, the guidance angle should be at -pi/2
            # w.r.t. the robot's forward axis when dist_error == 0.  If
            # dist_error is positive (meaning we are too close to the
            # landmarks) then the ideal guidance angle should be in the range
            # [-pi, -pi/2].  Otherwise, the ideal guidance angle should be in
            # the range [0, -pi/2].  We will use dist_error to scale linearly
            # within these ranges as we compute the ideal guidance angle.
            pi_2 = pi / 2.0
            ideal_guidance_angle = -pi_2 - pi_2 * dist_error
            action_print = "PROPORTIONAL CONTROL"

            # Now define the angle_error
            angle_error = get_smallest_signed_angular_difference(
                                                        self.guidance_angle,
                                                        ideal_guidance_angle)

            #twist = self.diff_to_twist_bang_bang(angle_error)
            twist = self.diff_to_twist_smooth(angle_error)

        if visualize:
            print("\t{}".format(action_print))

        return twist

    def diff_to_twist_bang_bang(self, diff):
        """ Given an angular difference, compute a twist to reduce it."""
        twist = Twist()
        if diff > 0:
            twist.angular = self.angular_speed
        else:
            twist.angular = -self.angular_speed

        if self.no_target_turn_on_spot:
            if fabs(diff) < pi/20.0:
                twist.linear = self.linear_speed
            else:
                twist.linear = 0
        else:
            twist.linear = self.linear_speed

        return twist

    def diff_to_twist_smooth(self, diff):
        """ Given an angular difference, compute a twist to reduce it."""
        """ WORKS AND IS SIMPLE, BUT REQUIRES A HIGH ANGULAR SPEED THAT
        A REAL ROBOT MAY NOT BE ABLE TO ACHIEVE """
        twist = Twist()
        twist.linear = self.linear_speed * cos(diff)
        twist.angular = self.angular_speed * sin(diff)
        return twist