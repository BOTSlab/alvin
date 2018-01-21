""" A fork of BumpController that REMOVES odometry-based homing to bump pucks towards the closest pair (or solitary) landmark. """

from controller import Controller
from math import atan2, exp, fabs, cos, sin, pi, sqrt, acos
from numpy import sign
from random import uniform
from common import *
from common.angles import get_smallest_angular_difference, \
                          get_smallest_signed_angular_difference
from configsingleton import ConfigSingleton
from controllers.distance_point_segment import *
#from controllers.extract_blobs_closest_points import *

# Our mechanism for selectively importing pyglet/GUI-related stuff.
import gui_setting
if gui_setting.use:
    from common.drawing import draw_ray, draw_segment_wrt_robot

class BumpController(Controller):

    def __init__(self, robot, puck_mask):
        """
        puck_mask -- The mask for pucks this controller recognizes
        """
        self.puck_mask = puck_mask

        config = ConfigSingleton.get_instance()

        self.linear_speed = config.getfloat("BumpController", 
                                            "linear_speed")
        self.angular_speed = config.getfloat("BumpController",
                                             "angular_speed")
        self.slow_factor = config.getfloat("BumpController",
                                           "slow_factor")
        self.puck_dist_threshold = config.getfloat("BumpController",
                                           "puck_dist_threshold")
        self.robot_react_range = config.getfloat("BumpController",
                                           "robot_react_range")
        self.wall_react_range = config.getfloat("BumpController",
                                           "wall_react_range")
        #self.target_angle_bias = config.getfloat("BumpController",
        #                                   "target_angle_bias")
        self.inner_exclude_angle = config.getfloat("BumpController",
                                           "inner_exclude_angle")
        self.outer_exclude_angle = config.getfloat("BumpController",
                                           "outer_exclude_angle")
        self.use_tracking = config.getboolean("BumpController", "use_tracking")
        self.tracking_threshold = config.getfloat("BumpController",
                                           "tracking_threshold")
        self.lmark_ideal_range = config.getfloat("BumpController",
                                           "lmark_ideal_range")

        # One of: "SIMPLE" "BACK_RIGHT" "FRONT_POSITIVE" "BOTH"
        self.pair_condition = config.get("BumpController", "pair_condition")

        # One of: "SIMPLE", "ON RIGHT", "ONLY_IF_NO_PAIR"
        self.single_condition = config.get("BumpController", "single_condition")

        # One of: "SIMPLE", "ALPHA", "BETA", "BOTH"
        self.puck_condition = config.get("BumpController", "puck_condition")

        self.wall_turn_on_spot = config.getboolean("BumpController",
                                           "wall_turn_on_spot")
        self.no_target_turn_on_spot = config.getboolean("BumpController",
                                           "no_target_turn_on_spot")

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

            self.closest_pair   -- The closest landmark pair representing a
                                   segment.  'None' if no viable pair exists.
            self.guidance_vec   -- Vector from the robot to the nearest point
                                   on the closest line segment, or to the
                                   closest individual landmark (if
                                   self.closest_pair == None).
            self.guidance_angle -- Angle of self.guidance_vec.
        """

        #image = sensor_dump.upper_image

        # Get a list of all landmark pairs for the inter-landmark distance is
        # less than the threshold (meaning that the line segment between them
        # should be filled in).  The result will be list of tuples representing
        # the coordinates of the landmark pairs in the robot reference frame.
        lmark_pairs = []
        lmark_scan = sensor_dump.landmarks
        nLmarks = len(lmark_scan.landmarks)
        for i in range(nLmarks):
            it = lmark_scan.landmarks[i].angle
            ir = lmark_scan.landmarks[i].distance

            for j in range(i+1, nLmarks):
                jt = lmark_scan.landmarks[j].angle
                jr = lmark_scan.landmarks[j].distance

                inter_lmark_dist = sqrt(ir**2 + jr**2 - 2*ir*jr*
                    cos(get_smallest_angular_difference(it, jt)))
                # The (x, y) coordinates for each landmark, a and b.
                # Note that we may switch the roles of a and b below.
                a = (ir * cos(it), ir * sin(it))
                b = (jr * cos(jt), jr * sin(jt))

                # Each landmark pair is ordered according to x.  The
                # first (a) has a lower x-coordinate then the second (b)
                if a[0] >= b[0]:
                    # Swap a and b.
                    tmp = a
                    a = b
                    b = tmp

                if (self.pair_condition == "SIMPLE"
                   and inter_lmark_dist < self.lmark_pair_dist):
                    lmark_pairs.append((a, b))
                    if visualize:
                        draw_segment_wrt_robot(robot, a, b,
                                           color=(127,127,127), width=3)

                elif (self.pair_condition == "BACK_RIGHT"
                   and inter_lmark_dist < self.lmark_pair_dist
                   and a[1] < 0):
                    lmark_pairs.append((a, b))
                    if visualize:
                        draw_segment_wrt_robot(robot, a, b,
                                           color=(127,127,127), width=3)

                elif (self.pair_condition == "FRONT_POSITIVE"
                   and inter_lmark_dist < self.lmark_pair_dist
                   and b[0] > 0):
                    lmark_pairs.append((a, b))
                    if visualize:
                        draw_segment_wrt_robot(robot, a, b,
                                           color=(127,127,127), width=3)

                elif (self.pair_condition == "BOTH"
                   and inter_lmark_dist < self.lmark_pair_dist
                   and a[1] < 0   # The back landmark (a) should lie on
                   and b[0] > 0): # landmark (b) should have positive x.
                    lmark_pairs.append((a, b))
                    if visualize:
                        draw_segment_wrt_robot(robot, a, b,
                                           color=(127,127,127), width=3)

#        MORE REALISITIC, BUT SLOWER
#
#        for i in range(image.n_rows):
#            if image.masks[i] & ANY_LANDMARK_MASK != 0:
#                (ixr, iyr) = image.calib_array[i,2], image.calib_array[i,3]
#                it = image.calib_array[i,4]
#                ir = image.calib_array[i,5]
#
#                for j in range(i+1,image.n_rows):
#                    if image.masks[j] & ANY_LANDMARK_MASK != 0:
#                        (jxr, jyr) = image.calib_array[j,2], image.calib_array[j,3]
#                        jt = image.calib_array[j,4]
#                        jr = image.calib_array[j,5]
#
#                        inter_lmark_dist = sqrt(ir**2 + jr**2 - 2*ir*jr*
#                            cos(get_smallest_angular_difference(it, jt)))
#                        # The (x, y) coordinates for each landmark, a and b.
#                        # Note that we may switch the roles of a and b below.
#                        a = (ir * cos(it), ir * sin(it))
#                        b = (jr * cos(jt), jr * sin(jt))
#
#                        # Each landmark pair is ordered according to x.  The
#                        # first (a) has a lower x-coordinate then the second (b)
#                        if a[0] >= b[0]:
#                            # Swap a and b.
#                            tmp = a
#                            a = b
#                            b = tmp
#
#                        if (self.pair_condition == "SIMPLE"
#                           and inter_lmark_dist < self.lmark_pair_dist):
#                            lmark_pairs.append((a, b))
#                            if visualize:
#                                draw_segment_wrt_robot(robot, a, b,
#                                                   color=(127,127,127), width=3)
#
#                        elif (self.pair_condition == "BACK_RIGHT"
#                           and inter_lmark_dist < self.lmark_pair_dist
#                           and a[1] < 0):
#                            lmark_pairs.append((a, b))
#                            if visualize:
#                                draw_segment_wrt_robot(robot, a, b,
#                                                   color=(127,127,127), width=3)
#
#                        elif (self.pair_condition == "FRONT_POSITIVE"
#                           and inter_lmark_dist < self.lmark_pair_dist
#                           and b[0] > 0):
#                            lmark_pairs.append((a, b))
#                            if visualize:
#                                draw_segment_wrt_robot(robot, a, b,
#                                                   color=(127,127,127), width=3)
#
#                        elif (self.pair_condition == "BOTH"
#                           and inter_lmark_dist < self.lmark_pair_dist
#                           and a[1] < 0   # The back landmark (a) should lie on
#                           and b[0] > 0): # landmark (b) should have positive x.
#                            lmark_pairs.append((a, b))
#                            if visualize:
#                                draw_segment_wrt_robot(robot, a, b,
#                                                   color=(127,127,127), width=3)

        # Find the closest landmark pair to the robot.
        closest_pair_d = float('inf')
        self.closest_pair = None
        self.guidance_vec = None
        self.guidance_angle = None
        for pair in lmark_pairs:
            proj_vec = distance_point_segment_projection((0, 0),
                               pair[0], pair[1], False)
            #d = distance_point_segment((0, 0), pair[0], pair[1], False)
            d = sqrt(proj_vec[0]**2 + proj_vec[1]**2)
            if d < closest_pair_d:
                closest_pair_d = d
                self.closest_pair = pair
                self.guidance_vec = proj_vec
                self.guidance_angle = atan2(proj_vec[1], proj_vec[0])

        # See if guidance should come from the closest single landmark.
        check_single_landmark = True
        if (self.single_condition == "ONLY_IF_NO_PAIR"
           and self.closest_pair == None):
            check_single_landmark = False

        if check_single_landmark:
            for i in range(nLmarks):
                a = lmark_scan.landmarks[i].angle
                r = lmark_scan.landmarks[i].distance

                if ((self.single_condition == "SIMPLE" or self.single_condition == "ONLY_IF_NO_PAIR") and r < closest_pair_d):
                    closest_pair_d = r
                    self.closest_pair = None
                    self.guidance_vec = (r * cos(a), r * sin(a))
                    self.guidance_angle = a

                elif (self.single_condition == "ON_RIGHT"
                   and a < 0 # Require single to be on right.
                   and r < closest_pair_d):
                    closest_pair_d = r
                    self.closest_pair = None
                    self.guidance_vec = (r * cos(a), r * sin(a))
                    self.guidance_angle = a
#            REALISTIC BUT SLOWER
#            for i in range(image.n_rows):
#                if image.masks[i] & ANY_LANDMARK_MASK != 0:
#                    (xr, yr) = image.calib_array[i,2], image.calib_array[i,3]
#                    a = image.calib_array[i,4]
#                    r = image.calib_array[i,5]
#
#                    if ((self.single_condition == "SIMPLE" or self.single_condition == "ONLY_IF_NO_PAIR") and r < closest_pair_d):
#                        closest_pair_d = r
#                        self.closest_pair = None
#                        self.guidance_vec = (r * cos(a), r * sin(a))
#                        self.guidance_angle = a
#
#                    elif (self.single_condition == "ON_RIGHT"
#                       and a < 0 # Require single to be on right.
#                       and r < closest_pair_d):
#                        closest_pair_d = r
#                        self.closest_pair = None
#                        self.guidance_vec = (r * cos(a), r * sin(a))
#                        self.guidance_angle = a
    
        """
        if self.closest_pair != None:
            print "PAIR"
        elif self.guidance_vec != None:
            print "SINGLE"
        else:
            print "NO LANDMARK"
        """

        if visualize and self.guidance_vec != None:
            if self.closest_pair == None:
                draw_segment_wrt_robot(robot, (0,0), self.guidance_vec,
                    color=(255,255,0), width=3)
            else:
                draw_segment_wrt_robot(robot, self.closest_pair[0],
                    self.closest_pair[1], color=(255,255,255), width=3)

    def process_pucks(self, robot, sensor_dump, visualize=False):
        """ If self.target_pos is set (not None) then process the puck pixels
        in the vicinity of this target.  If it is not set, consider all puck
        pixels.
        """

        image = sensor_dump.lower_image

        # If self.target_pos is set, we modify the image to delete the masks of
        # all pixels not within a threshold distance of the target.
        if self.use_tracking and self.target_pos != None:
            for i in range(image.n_rows):
                if image.masks[i] & self.puck_mask != 0:
                    (xr, yr) = image.calib_array[i,2], image.calib_array[i,3]
                    dx = self.target_pos[0] - xr
                    dy = self.target_pos[1] - yr
                    distance_to_target = sqrt(dx*dx + dy*dy)
                    if distance_to_target > self.tracking_threshold:
                        image.masks[i] = 0

        # Now we look at all puck pixels and calculate the perpendicular dist
        # from each to the segment pair identified above.  We set the target
        # to the puck with the largest d (i.e. the largest-closest distance).
        new_target_selected = False
        if self.closest_pair != None:
            largest_d = 0
            for i in range(image.n_rows):
                if image.masks[i] & self.puck_mask != 0:
                    (xr, yr) = image.calib_array[i,2], image.calib_array[i,3]
                    pt = image.calib_array[i,4]
                    pr = image.calib_array[i,5]

                    puck_vec = (pr * cos(pt), pr * sin(pt))
                    proj_vec = distance_point_segment_projection(puck_vec,
                           self.closest_pair[0], self.closest_pair[1], True)
                    if proj_vec == None:
                        continue
                    d = sqrt(distance_squared(puck_vec, proj_vec))
                    proj_vec_angle = atan2(proj_vec[1], proj_vec[0])

                    if visualize:
                        draw_segment_wrt_robot(robot, puck_vec, proj_vec,
                                               color=(255,0,255), width=3)

                    condition_okay = True
                    if (self.puck_condition == "SIMPLE"):
                        pass

                    elif (self.puck_condition == "ALPHA"
                      and pt <= self.guidance_angle + self.inner_exclude_angle):
                        condition_okay = False

                    elif (self.puck_condition == "BETA"
                      and pt >= self.guidance_angle + self.outer_exclude_angle):
                        condition_okay = False

                    elif (self.puck_condition == "BOTH"
                      and (pt <= self.guidance_angle + self.inner_exclude_angle
                      or pt >= self.guidance_angle + self.outer_exclude_angle)):
                        condition_okay = False

                    if (condition_okay
                       and d > self.puck_dist_threshold
                       and pt > proj_vec_angle # prevent influence of pucks on 
                                               # the far side of segment 
                       and d > largest_d):
                        largest_d = d
                        self.target_pos = (xr, yr)
                        new_target_selected = True
                        #    (pr, pt + self.target_angle_bias)
        else:
            # We are being guided only by the closest landmark
            largest_d = 0
            for i in range(image.n_rows):
                if image.masks[i] & self.puck_mask != 0:
                    (xr, yr) = image.calib_array[i,2], image.calib_array[i,3]
                    pt = image.calib_array[i,4]
                    pr = image.calib_array[i,5]

                    puck_vec = (pr * cos(pt), pr * sin(pt))
                    d = sqrt(distance_squared(puck_vec, self.guidance_vec))

                    if visualize:
                        draw_segment_wrt_robot(robot, puck_vec, 
                            self.guidance_vec, color=(255,0,255), width=3)
                        
                    condition_okay = True
                    if (self.puck_condition == "SIMPLE"):
                        pass

                    elif (self.puck_condition == "ALPHA"
                      and pt <= self.guidance_angle + self.inner_exclude_angle):
                        condition_okay = False

                    elif (self.puck_condition == "BETA"
                      and pt >= self.guidance_angle + self.outer_exclude_angle):
                        condition_okay = False

                    elif (self.puck_condition == "BOTH"
                      and (pt <= self.guidance_angle + self.inner_exclude_angle
                      or pt >= self.guidance_angle + self.outer_exclude_angle)):
                        condition_okay = False

                    if (condition_okay
                       and d > self.puck_dist_threshold
                       and d > largest_d):
                        largest_d = d
                        self.target_pos = (xr, yr)
                        new_target_selected = True
                        #    (pr, pt + self.target_angle_bias)

        if not new_target_selected:
            self.target_pos = None

    def act(self, robot, sensor_dump, visualize=False):
        """ Set wheel speeds according to processed sensor data. """

        image = sensor_dump.lower_image

        # Check if there are no landmarks in view, but there are wall pixels we
        # can use for guidance.
        react_to_wall = False
        react_to_wall_angle = None
#        if self.guidance_vec == None:
        closest_wall_angle = None
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
        if react_to_robot:
            # Turn slow in the open direction
            twist.linear = self.slow_factor * self.linear_speed
            twist.angular = self.angular_speed
            if visualize:
                print "ROBOT"
                draw_ray(robot, react_to_robot_angle, color=(255,0,0))

        elif react_to_wall:
            if visualize:
                print "WALL"
                draw_ray(robot, react_to_wall_angle, color=(255,255,0))
            #twist = self.curve_to_angle(pi/2, react_to_wall_angle, self.wall_turn_on_spot)
            twist.linear = uniform(-0.1, 0.1) * self.linear_speed
            twist.angular = -self.angular_speed

        elif self.guidance_vec == None:
            # No landmarks in view AND no walls in view.  Just go straight.
            twist.linear = self.linear_speed
            if visualize:
                print "STRAIGHT (NO LANDMARKS OR WALLS IN VIEW)"

        elif self.target_pos != None and dist_to_lmarks > self.puck_dist_threshold:
            ##twist.linear = self.linear_speed * sign(self.target_pos[0])
            ##twist.angular = self.angular_speed * sign(self.target_pos[1])

            #twist.linear = self.linear_speed * 1.0 * self.target_pos[0]
            #twist.angular = self.angular_speed * 20.0 * self.target_pos[1]
            #twist = self.cap_twist(twist)

            angle = atan2(self.target_pos[1], self.target_pos[0])
            twist.linear = self.linear_speed * cos(angle)
            twist.angular = self.angular_speed * sin(angle)

            if visualize:
                print "TARGETING"
                draw_segment_wrt_robot(robot, (0, 0), 
                                       (self.target_pos[0], self.target_pos[1]),
                                       color=(255, 255, 255), width=3)
        else:

            #twist = self.curve_to_angle(-pi/2, self.guidance_angle, self.no_target_turn_on_spot)
            dist = sqrt(self.guidance_vec[0]**2 + self.guidance_vec[1]**2)
            gamma = 0.25
            twist = self.curve_to_angle(-pi/2 + gamma, self.guidance_angle, self.no_target_turn_on_spot)
            if dist < self.lmark_ideal_range:
                twist = self.curve_to_angle(-pi/2 - gamma, self.guidance_angle, self.no_target_turn_on_spot)
                if visualize:
                    print "CURVING OUT"
            else:
                twist = self.curve_to_angle(-pi/2 + gamma, self.guidance_angle, self.no_target_turn_on_spot)
                if visualize:
                    print "CURVING IN"
            """
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
            print dist_error

            # But we also have to consider the current angle w.r.t. the
            # guidance angle.  Ideally, the guidance angle should be at -pi/2
            # w.r.t. the robot's forward axis when dist_error == 0.  If
            # dist_error is positive (meaning we are too close to the
            # landmarks) then the ideal guidance angle should be in the range
            # [-pi, -pi/2].  Otherwise, the ideal guidance angle should be in
            # the range [0, -pi/2].  We will use dist_error to scale linearly
            # within these ranges as we compute the ideal guidance angle.
            pi_2 = pi / 2.0
            if dist_error > 0:
                ideal_guidance_angle = -pi_2 - pi_2 * dist_error
            else:
                ideal_guidance_angle =  pi_2 * dist_error

            # Now define the angle_error
            #angle_error = ideal_guidance_angle - self.guidance_angle
            angle_error = get_smallest_signed_angular_difference(
                                                        self.guidance_angle,
                                                        ideal_guidance_angle)

            twist.linear = self.linear_speed * cos(angle_error)
            twist.angular = self.angular_speed * sin(angle_error)
            #twist.angular = self.angular_speed * (2/(1+exp(-angle_error)) - 1)
            """

        return twist

    def curve_to_angle(self, desired_angle, actual_angle, turn_on_spot):
        twist = Twist()

        diff = get_smallest_signed_angular_difference(desired_angle,
                                                      actual_angle)
        if diff > 0:
            twist.angular = - self.angular_speed
        else:
            twist.angular = self.angular_speed

        if turn_on_spot:
            if fabs(diff) < pi/10.0:
                twist.linear = self.linear_speed
        else:
            twist.linear = self.linear_speed

        return twist

    def cap_twist(self, twist):
        if twist.linear > 0:
            twist.linear = min(self.linear_speed, twist.linear)
        if twist.linear < 0:
            twist.linear = max(-self.linear_speed, twist.linear)
        if twist.angular > 0:
            twist.angular = min(self.angular_speed, twist.angular)
        if twist.angular < 0:
            twist.angular = max(-self.angular_speed, twist.angular)

        return twist
