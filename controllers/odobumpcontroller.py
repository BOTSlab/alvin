""" A fork of LineBumpController that uses odometry-based homing to bump pucks towards the closest pair (or solitary) landmark. """

from .controller import Controller
from math import atan2, fabs, cos, sin, pi, sqrt, acos
from numpy import sign
from random import random, gauss
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

class OdoBumpController(Controller):

    def __init__(self, robot, puck_mask):
        """
        puck_mask -- The mask for pucks this controller recognizes
        """
        self.puck_mask = puck_mask

        config = ConfigSingleton.get_instance()

        self.linear_speed = config.getfloat("OdoBumpController", 
                                            "linear_speed")
        self.angular_speed = config.getfloat("OdoBumpController",
                                             "angular_speed")
        self.slow_factor = config.getfloat("OdoBumpController",
                                           "slow_factor")
        self.puck_dist_threshold = config.getfloat("OdoBumpController",
                                           "puck_dist_threshold")
        self.robot_react_range = config.getfloat("OdoBumpController",
                                           "robot_react_range")
        self.target_angle_bias = config.getfloat("OdoBumpController",
                                           "target_angle_bias")
        self.inner_exclude_angle = config.getfloat("OdoBumpController",
                                           "inner_exclude_angle")
        self.outer_exclude_angle = config.getfloat("OdoBumpController",
                                           "outer_exclude_angle")
        self.odo_home_threshold = config.getfloat("OdoBumpController",
                                           "odo_home_threshold")
        self.odo_home_factor = config.getfloat("OdoBumpController",
                                           "odo_home_factor")
        self.odo_home_timeout = config.getint("OdoBumpController",
                                           "odo_home_timeout")

        # One of: "NO_ODO", "DISTANCE_TRAVELLED", "COUNT":
        self.odo_condition = config.get("OdoBumpController", "odo_condition")

        # One of: "SIMPLE" "BACK_RIGHT" "FRONT_POSITIVE" "BOTH"
        self.pair_condition = config.get("OdoBumpController", "pair_condition")

        # One of: "SIMPLE", "ON RIGHT", "ONLY_IF_NO_PAIR"
        self.single_condition = config.get("OdoBumpController", "single_condition")

        # One of: "SIMPLE", "ALPHA", "BETA", "BOTH"
        self.puck_condition = config.get("OdoBumpController", "puck_condition")


        # lmark_dist_threshold is a bit different.  We set it depending upon
        # the configuration of canned landmarks.
        canned_landmarks_name = config.get("AlvinSim", "canned_landmarks_name")
        if canned_landmarks_name == "ONE_CENTRAL":
            self.lmark_dist_threshold = 0

        elif canned_landmarks_name == "L_SHAPE":
            self.lmark_dist_threshold = 480

        elif canned_landmarks_name == "M_SHAPE":
            self.lmark_dist_threshold = 230

        elif canned_landmarks_name == "C_SHAPE":
            self.lmark_dist_threshold = 215

        self.robot = robot

        # Possible states:
        #   "SENSE"    -- Process sensor data and select target point.
        #   "ODO_HOME" -- Move to target point via odometry-based homing.
        self.state = "SENSE"

        # Range and bearing of the target in the odometry reference frame.
        self.target_range_bearing = None

        # ODO_HOME-related
        self.odo_home_start_pos = None
        self.odo_home_start_distance = None
        self.odo_home_count = 0

    def react(self, robot, sensor_dump, visualize=False):
        self.process_landmarks(robot, sensor_dump, visualize)

        (x, y, theta) = sensor_dump.odo_pose

        if self.state == "SENSE":
            # In this state we consider all visible pucks and will set
            # self.target_range_bearing if a viable puck pixel is visible.
            self.target_range_bearing = None
            if self.guidance_vec != None:
                self.process_pucks(robot, sensor_dump, visualize)

            if self.target_range_bearing != None:
                # Get the goal position in the odometry reference frame.
                r = self.target_range_bearing[0]
                a = self.target_range_bearing[1]
                self.odo_goal_x = x + r * cos(theta + a)
                self.odo_goal_y = y + r * sin(theta + a)

                if self.odo_condition != "NO_ODO":
                    self.state = "ODO_HOME"

                # NEW
                self.odo_home_start_pos = x, y
                self.odo_home_start_distance = r

                # OLD
                self.odo_home_count = 0

        elif self.state == "ODO_HOME":
            assert self.target_range_bearing != None

            # First estimate distance travelled to this point.
            dx = x - self.odo_home_start_pos[0]
            dy = y - self.odo_home_start_pos[1]
            distance_travelled = sqrt(dx*dx + dy*dy)
            
            # Now estimate distance remaining to the goal.
            dx = x - self.odo_goal_x
            dy = y - self.odo_goal_y
            distance_to_goal = sqrt(dx*dx + dy*dy)

            if distance_to_goal < self.odo_home_threshold:
                self.state = "SENSE"

            # NEW:
            if self.odo_condition == "DISTANCE_TRAVELLED":
                if (self.odo_home_factor * distance_travelled > 
                   self.odo_home_start_distance):
                    self.state = "SENSE"

            elif self.odo_condition == "COUNT":
                self.odo_home_count += 1
                if (self.odo_home_count > self.odo_home_timeout):
                    self.state = "SENSE"

            else:
                assert False
        else:
            assert False

        #print self.state

        if visualize:
            if self.state == "ODO_HOME":
                robot.shape.color = 255, 155, 0
            else:
                robot.shape.color = 0, 255, 0

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

        # Reduce blobs all representing one landmark to single points.
        #image = extract_blobs_closest_points(robot, sensor_dump.upper_image,
        #                                     ANY_LANDMARK_MASK)
        image = sensor_dump.upper_image

        # Get a list of all landmark pairs for the inter-landmark distance is
        # less than the threshold (meaning that the line segment between them
        # should be filled in).  The result will be list of tuples representing
        # the coordinates of the landmark pairs in the robot reference frame.
        lmark_pairs = []
        for i in range(image.n_rows):
            if image.masks[i] & ANY_LANDMARK_MASK != 0:
                (ixr, iyr) = image.calib_array[i,2], image.calib_array[i,3]
                it = image.calib_array[i,4]
                ir = image.calib_array[i,5]

                for j in range(i+1,image.n_rows):
                    if image.masks[j] & ANY_LANDMARK_MASK != 0:
                        (jxr, jyr) = image.calib_array[j,2], image.calib_array[j,3]
                        jt = image.calib_array[j,4]
                        jr = image.calib_array[j,5]

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
                           and inter_lmark_dist < self.lmark_dist_threshold):
                            lmark_pairs.append((a, b))
                            if visualize:
                                draw_segment_wrt_robot(robot, a, b,
                                                   color=(127,127,127), width=3)

                        elif (self.pair_condition == "BACK_RIGHT"
                           and inter_lmark_dist < self.lmark_dist_threshold
                           and a[1] < 0):
                            lmark_pairs.append((a, b))
                            if visualize:
                                draw_segment_wrt_robot(robot, a, b,
                                                   color=(127,127,127), width=3)

                        elif (self.pair_condition == "FRONT_POSITIVE"
                           and inter_lmark_dist < self.lmark_dist_threshold
                           and b[0] > 0):
                            lmark_pairs.append((a, b))
                            if visualize:
                                draw_segment_wrt_robot(robot, a, b,
                                                   color=(127,127,127), width=3)

                        elif (self.pair_condition == "BOTH"
                           and inter_lmark_dist < self.lmark_dist_threshold
                           and a[1] < 0   # The back landmark (a) should lie on
                           and b[0] > 0): # landmark (b) should have positive x.
                            lmark_pairs.append((a, b))
                            if visualize:
                                draw_segment_wrt_robot(robot, a, b,
                                                   color=(127,127,127), width=3)

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

        if visualize:
            if self.closest_pair != None:
                draw_segment_wrt_robot(robot, self.closest_pair[0],
                        self.closest_pair[1], color=(255,255,255), width=3)

        # See if guidance should come from the closest single landmark.
        check_single_landmark = True
        if (self.single_condition == "ONLY_IF_NO_PAIR"
           and self.closest_pair == None):
            check_single_landmark = False

        if check_single_landmark:
            for i in range(image.n_rows):
                if image.masks[i] & ANY_LANDMARK_MASK != 0:
                    (xr, yr) = image.calib_array[i,2], image.calib_array[i,3]
                    a = image.calib_array[i,4]
                    r = image.calib_array[i,5]

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
    
        """
        if self.closest_pair != None:
            print "PAIR"
        elif self.guidance_vec != None:
            print "SINGLE"
        else:
            print "NO LANDMARK"
        """

        if visualize:
            if self.guidance_vec != None:
                draw_segment_wrt_robot(robot, (0,0), self.guidance_vec,
                                       color=(255,255,0), width=3)


    def process_pucks(self, robot, sensor_dump, visualize=False):
        """ Process the visible pucks and set self.target_range_bearing if
        there is a viable target.
        """

        image = sensor_dump.lower_image

        # Now we look at all puck pixels and calculate the perpendicular dist
        # from each to the segment pair identified above.  We set the target
        # to the puck with the largest d (i.e. the largest-closest distance).
        assert self.target_range_bearing == None
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
                        self.target_range_bearing = \
                            (pr, pt + self.target_angle_bias)
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
                        
                    if (pt > self.guidance_angle #+ pi/4
                       and (pt - self.guidance_angle) < self.outer_exclude_angle
                       and d > self.puck_dist_threshold
                       and d > largest_d):
                        largest_d = d
                        self.target_range_bearing = \
                            (pr, pt + self.target_angle_bias)

    def act(self, robot, sensor_dump, visualize=False):
        """ Set wheel speeds according to processed sensor data. """

        image = sensor_dump.lower_image

        # Check if there are no landmarks in view, but there are wall pixels we
        # can use for guidance.
        react_to_wall = False
        react_to_wall_angle = None
        if self.guidance_vec == None:
            closest_wall_angle = None
            closest_wall_range = float('inf')
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

        twist = Twist()
        if react_to_robot:
            # Turn slow in the open direction
            twist.linear = self.slow_factor * self.linear_speed
            twist.angular = self.angular_speed
            if visualize:
                draw_ray(robot, react_to_robot_angle, color=(255,0,0))

        elif react_to_wall:
            if visualize:
                draw_ray(robot, react_to_wall_angle, color=(255,255,0))
            twist = self.curve_to_angle(pi/2, react_to_wall_angle)

        elif self.guidance_vec == None:
            # No landmarks in view AND no walls in view.  Just go straight.
            twist.linear = self.linear_speed

        elif (self.state == "ODO_HOME"
             or (self.odo_condition=="NO_ODO"
                 and self.target_range_bearing!=None)):
            (x, y, theta) = sensor_dump.odo_pose
            c = cos(theta)
            s = sin(theta)
            goal_Rx = c * (self.odo_goal_x - x) + s * (self.odo_goal_y - y)
            goal_Ry = -s * (self.odo_goal_x - x) + c * (self.odo_goal_y - y)

            #twist.linear = 0.1 * goal_Rx
            #twist.angular = 0.2 * goal_Ry
            twist.linear = self.linear_speed * sign(goal_Rx)
            twist.angular = self.angular_speed * sign(goal_Ry)
            
            if visualize:
                draw_segment_wrt_robot(robot, (0, 0), (goal_Rx, goal_Ry),
                                       color=(255,0,255), width=3)

        else:
            twist = self.curve_to_angle(-pi/2, self.guidance_angle)

        return twist

    def curve_to_angle(self, desired_angle, actual_angle):
        twist = Twist()

        diff = get_smallest_signed_angular_difference(desired_angle,
                                                      actual_angle)
        if diff > 0:
            twist.angular = - self.angular_speed
        else:
            twist.angular = self.angular_speed
        #if fabs(diff) < pi/10.0:
        twist.linear = self.linear_speed

        return twist
