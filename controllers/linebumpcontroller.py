""" Forms patterns with respect to landmarks by bumping pucks towards pairs of landmarks which define individual line segments. """

from controller import Controller
from math import atan2, fabs, cos, sin, pi, sqrt, acos
from random import random, normalvariate
from common import *
from common.drawing import draw_line, draw_segment_wrt_robot
from common.angles import get_smallest_angular_difference, \
                          get_smallest_signed_angular_difference
from configsingleton import ConfigSingleton
from controllers.landmarkfilter import thin_scan
from controllers.distance_point_segment import *

class LineBumpController(Controller):

    def __init__(self, robot, puck_mask):
        """
        puck_mask -- The mask for pucks this controller recognizes
        """
        self.puck_mask = puck_mask

        config = ConfigSingleton.get_instance()

        self.linear_speed = config.getfloat("LineBumpController", 
                                            "linear_speed")
        self.angular_speed = config.getfloat("LineBumpController",
                                             "angular_speed")
        self.slow_factor = config.getfloat("LineBumpController",
                                           "slow_factor")
        self.lmark_dist_threshold = config.getfloat("LineBumpController",
                                           "lmark_dist_threshold")
        self.puck_dist_threshold = config.getfloat("LineBumpController",
                                           "puck_dist_threshold")
        self.robot_react_range = config.getfloat("LineBumpController",
                                           "robot_react_range")

        self.robot = robot

    def react(self, robot, sensor_suite, visualize=False):

        pscan = sensor_suite.range_scan
        np = pscan.NUMBER_POINTS

        # Reduce multiple spans all representing one landmark to single points.
        lscan = thin_scan(robot, sensor_suite.landmark_scan, ANY_LANDMARK_MASK)
        nl = lscan.NUMBER_POINTS

        # Get a list of all landmark pairs for the inter-landmark distance is
        # less than the threshold (meaning that the line segment between them
        # should be filled in).  The result will be list of tuples representing
        # the coordinates of the landmark pairs in the robot reference frame.
        lmark_pairs = []
        for i in range(nl):
            if lscan.masks[i] & ANY_LANDMARK_MASK != 0:
                ir = lscan.ranges[i] # i_range
                it = lscan.angles[i] # i_theta
                for j in range(i+1,nl):
                    if lscan.masks[j] & ANY_LANDMARK_MASK != 0:
                        jr = lscan.ranges[j] # j_range
                        jt = lscan.angles[j] # j_theta
                        inter_lmark_dist = sqrt(ir**2 + jr**2 - 2*ir*jr*
                            cos(get_smallest_angular_difference(it, jt)))
                        # The (x, y) coordinates for each landmark, a and b.
                        # Note that we may switch the roles of a and b below.
                        a = (ir * cos(it), ir * sin(it))
                        b = (jr * cos(jt), jr * sin(jt))

                        if (inter_lmark_dist < self.lmark_dist_threshold):
                            # Each landmark pair is ordered according to x.
                            if a[0] < b[0]:
                                lmark_pairs.append((a, b))
                            else:
                                lmark_pairs.append((b, a))

                            if visualize:
                                draw_segment_wrt_robot(robot, a, b,
                                                   color=(127,127,127), width=3)

        # Find the closest landmark pair to the robot.
        closest_pair_d = float('inf')
        closest_pair = None
        guidance_vec = None
        guidance_angle = None
        for pair in lmark_pairs:
            proj_vec = distance_point_segment_projection((0, 0),
                               pair[0], pair[1], False)
            #d = distance_point_segment((0, 0), pair[0], pair[1], False)
            d = sqrt(proj_vec[0]**2 + proj_vec[1]**2)
            if d < closest_pair_d:
                closest_pair_d = d
                closest_pair = pair
                guidance_vec = proj_vec
                guidance_angle = atan2(proj_vec[1], proj_vec[0])

        if visualize:
            if closest_pair != None:
                draw_segment_wrt_robot(robot, closest_pair[0], closest_pair[1],
                                       color=(255,255,255), width=3)

        # There may be no viable pairs.  In which case, behaviour will be based
        # on the closest single landmark.
        closest_lmark_dist = float('inf')
        closest_lmark_angle = None
        if closest_pair == None:
            # Find the closest single landmark
            closest_lmark_dist = float('inf')
            for i in range(nl):
                if lscan.masks[i] & ANY_LANDMARK_MASK != 0:
                    r = lscan.ranges[i]
                    a = lscan.angles[i]
                    if r < closest_lmark_dist:
                        closest_lmark_dist = r
                        closest_lmark_angle = a
                        guidance_vec = (r * cos(a), r * sin(a))
                        guidance_angle = a

        if visualize:
            if guidance_vec != None:
                draw_segment_wrt_robot(robot, (0,0), guidance_vec,
                                       color=(255,255,0), width=3)

        # Now we look at all puck pixels and calculate the perpendicular dist
        # from each to the segment pair identified above.  We set target_angle
        # to the puck with the largest d (i.e. the largest-closest distance).
        target_angle = None
        #seg_vec = None
        if closest_pair != None:
            # A vector representing the direction of the segment pair
            #         (b.x - a.x, b.y - a.y)
            #seg_vec = (closest_pair[1][0] - closest_pair[0][0],
            #           closest_pair[1][1] - closest_pair[0][1])

            largest_d = 0
            for p in range(np):
                if pscan.masks[p] & self.puck_mask != 0:
                    pr = pscan.ranges[p]
                    pt = pscan.angles[p]
                    puck_vec = (pr * cos(pt), pr * sin(pt))

                    proj_vec = distance_point_segment_projection(puck_vec,
                                       closest_pair[0], closest_pair[1], False)

                    if visualize:
                        draw_segment_wrt_robot(robot, puck_vec, proj_vec,
                                               color=(255,0,255), width=3)
                        
                    d = sqrt(distance_squared(puck_vec, proj_vec))
                    #d = distance_point_segment(puck_vec, closest_pair[0],
                    #                           closest_pair[1], False)

                    # Get the angle between the vector from the robot to the
                    # puck and the vector of the line segment.
                    #dot_product = puck_vec[0]*seg_vec[0] + \
                    #              puck_vec[1]*seg_vec[1]
                    puck_vec_length = sqrt(square(puck_vec[0]) + \
                                           square(puck_vec[1]))
                    #seg_vec_length = sqrt(square(seg_vec[0]) + \
                    #                      square(seg_vec[1]))
                    #angle_between = acos(dot_product / 
                    #                     (puck_vec_length*seg_vec_length))

                    proj_vec_angle = atan2(proj_vec[1], proj_vec[0])

                    if (pt > guidance_angle 
                       and (pt - guidance_angle) < 3*pi/4
                       and d > self.puck_dist_threshold
                       and pt > proj_vec_angle # prevent influence of pucks on 
                                               # the far side of segment 
                       and d > largest_d):
                        largest_d = d
                        target_angle = pt
        else:
            # We are being guided only by the closest landmark
            largest_d = 0
            for p in range(np):
                if pscan.masks[p] & self.puck_mask != 0:
                    pr = pscan.ranges[p]
                    pt = pscan.angles[p]
                    puck_vec = (pr * cos(pt), pr * sin(pt))

                    if visualize:
                        draw_segment_wrt_robot(robot, puck_vec, guidance_vec,
                                               color=(255,0,255), width=3)
                        
                    d = sqrt(distance_squared(puck_vec, guidance_vec))

                    if (pt > guidance_angle 
                       and (pt - guidance_angle) < 3*pi/4
                       and d > self.puck_dist_threshold
                       and d > largest_d):
                        largest_d = d
                        target_angle = pt

        # Check for the presence of another robot in the centre of the image.
        react_to_robot = False
        react_to_robot_angle = None
        np = len(pscan.ranges)
        for i in range(0, np):
            if (pscan.masks[i] == ROBOT_MASK and fabs(pscan.angles[i]) < pi/4
                and pscan.ranges[i] < self.robot_react_range):
                react_to_robot = True
                react_to_robot_angle = self.index_to_angle(pscan, i)

        twist = Twist()
        if react_to_robot:
            # Turn slow in the open direction
            twist.linear = self.slow_factor * self.linear_speed
            twist.angular = self.angular_speed
            draw_line(robot, pscan, react_to_robot_angle, color=(255,0,0))

        elif (target_angle == None):
            return self.curve_to_angle(guidance_angle, -pi/2)

        elif (target_angle <= 0):
            # Turn right
            twist.linear = self.linear_speed
            twist.angular = - self.angular_speed

        else:
            # Turn left
            twist.linear = self.linear_speed
            twist.angular = self.angular_speed

        if visualize:
            if target_angle != None:
                draw_line(robot, pscan, target_angle, color=(0,255,0))

        return twist

    def curve_to_angle(self, actual_angle, desired_angle):
        twist = Twist()

        diff = get_smallest_signed_angular_difference(desired_angle,
                                                      actual_angle)
        if diff > 0:
            # Turn right
            #twist.linear = self.linear_speed
            twist.angular = - self.angular_speed
        else:
            # Turn left
            #twist.linear = self.linear_speed
            twist.angular = self.angular_speed
        if fabs(diff) < pi/4.0:
            twist.linear = self.linear_speed

        return twist
