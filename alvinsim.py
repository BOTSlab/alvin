#!/usr/bin/env python

# Our mechanism for selectively importing pyglet/GUI-related stuff.
import gui_setting
if gui_setting.use:
    from alvinwindow import AlvinWindow

import pymunk, copy, os, sys, shutil, time

from math import pi, cos, sin, sqrt
from pymunk import Vec2d, ShapeFilter
from random import seed, randint, choice, uniform

from puck import Puck
from landmark import Landmark
from robot import Robot
from probe import Probe
from common import *
from sensors import RangeScan, RangeScanner, SensorSuite, SensorDump
from controllers.echocontroller import EchoController
from controllers.gaucirangecontroller import GauciRangeController
from controllers.bumpcontroller import BumpController
from controllers.simplebumpcontroller import SimpleBumpController
from configsingleton import ConfigSingleton
from analyzer import Analyzer

class AlvinSim(object):

    steps = 0

    # Set by window (GUI)
    allow_translation = True
    allow_rotation = True

    def __init__(self, config_file, trial_number):
        # Use the base of the config file's name as the name of the output dir
        output_dir_base = str.split(config_file, '.')[0]
        self.output_dir = output_dir_base + '/' + str(trial_number)
        print(self.output_dir)

        config = ConfigSingleton.get_instance(config_file)

        # Load parameters from config file.
        self.width_cm = config.getint("AlvinSim", "width_cm")
        self.height_cm = config.getint("AlvinSim", "height_cm")
        self.width = int(self.width_cm * CM_TO_PIXELS)
        self.height = int(self.height_cm * CM_TO_PIXELS)
        #print "width, height: {}, {}".format(self.width, self.height);
        self.circ_border = config.getboolean("AlvinSim", "circ_border")
        self.number_robots = config.getint("AlvinSim", "number_robots")
        self.number_pucks = config.getint("AlvinSim", "number_pucks")
        self.number_puck_kinds = config.getint("AlvinSim", "number_puck_kinds")
        self.number_steps = config.getint("AlvinSim", "number_steps")
        self.puck_ring = config.getboolean("AlvinSim", "puck_ring")
        self.puck_ring_radius = config.getint("AlvinSim", "puck_ring_radius")
        self.canned_landmarks_name = config.get("AlvinSim", 
                                               "canned_landmarks_name")
        self.puck_shape = config.get("AlvinSim", "puck_shape")
        self.lmark_pair_dist = config.getint("AlvinSim", "lmark_pair_dist")
        self.puck_kinds = list(range(self.number_puck_kinds))
        self.wall_thickness = config.getint("AlvinSim", "wall_thickness")
        self.capture_interval = config.getint("AlvinSim", "capture_interval")
        self.analyze = config.getboolean("AlvinSim", "analyze")
        self.capture_screenshots = config.getboolean("AlvinSim", "capture_screenshots")
        self.visualize_probes = config.getboolean("AlvinSim", "visualize_probes")
        self.controller_name = config.get("AlvinSim", "controller_name")
        self.render_skip = config.getint("AlvinSim", "render_skip")

        # build simulation environment
        self.env = pymunk.Space()
        self.env.damping = 0.0001 # 9999% of velocity is lost per second

        # Seed random number generator.
        seed(trial_number)

        # Create the walls, robots, pucks, and landmarks
        if self.circ_border:
            self.create_circ_border()
        else:
            self.create_rect_border()
        #self.create_random_walls()
        #self.create_one_wall()
        self.robots = []
        self.pucks = []
        self.landmarks = []
        self.probes = []
        self.create_robots()

        if self.puck_ring:
            self.create_pucks_ring()
        else:
            self.create_pucks_random()
        #self.create_immobile_pucks()
        self.create_canned_landmarks()
        if self.visualize_probes:
            self.create_probe_grid()

        # Prepare output directory.
        shutil.rmtree(self.output_dir, ignore_errors=True)
        os.makedirs(self.output_dir)

        if self.analyze:
            #init(self.output_dir)
            self.analyzer = Analyzer(self.output_dir, self.landmarks)

        # Setup to handle collisions
        #self.env.add_default_collision_handler().begin = self.collision_handler

        self.avg_dt = None

    def create_rect_border(self):
        env_b = self.env.static_body
        walls = []
        walls.append(self.create_wall(0, 0, self.width, 0))
        walls.append(self.create_wall(self.width, 0, self.width, self.height))
        walls.append(self.create_wall(self.width, self.height, 0, self.height))
        walls.append(self.create_wall(0, self.height, 0,0))
        for wall_shape in walls:
            wall_shape.filter = ShapeFilter(categories = WALL_MASK)
        self.env.add(walls)

    def get_random_pos_within_border(self, offset):
        if self.circ_border:
            cx, cy = self.width/2, self.height/2
            radius = min(cx, cy)
            #rho = uniform(0, radius)
            #theta = uniform(0, 2*pi)
            #return cx + rho*cos(theta), cy + rho*sin(theta)
            dist_from_centre = float('inf')
            while dist_from_centre > radius - offset:
                x = randint(offset, self.width - offset)
                y = randint(offset, self.height - offset)
                dx = x - cx
                dy = y - cy
                dist_from_centre = sqrt(dx**2 + dy**2)
            return (x, y)
        else:
            # Assumed to be rectangular.
            x = randint(offset, self.width - offset)
            y = randint(offset, self.height - offset)
            return (x, y)

    def create_circ_border(self):
        env_b = self.env.static_body
        walls = []
        # n is the number of divisions
        n = 50
        last_angle = 0
        cx = self.width/2
        cy = self.height/2
        radius = min(cx, cy) + self.wall_thickness
        for angle in [2*pi*i/n for i in range(1,n+1)]:
            x1 = cx + radius * cos(last_angle)
            y1 = cy + radius * sin(last_angle)
            x2 = cx + radius * cos(angle)
            y2 = cy + radius * sin(angle)
            last_angle = angle
            walls.append(self.create_wall(x1, y1, x2, y2))
        self.env.add(walls)

    def create_random_walls(self):
        # A few randomly distributed walls.
        random_walls = []
        n = randint(10, 20)
        #n = 0 # Open environment
        for i in range(n):
            x1, y1 = self.get_random_pos_within_border(0)
            angle = pi/2. * randint(0,3)
            length = randint(self.wall_thickness, self.width/2)
            x2, y2 = x1 + length*cos(angle), y1 + length*sin(angle)
            wall = self.create_wall(x1, y1, x2, y2)
            random_walls.append(wall)
        self.env.add(random_walls)

    def create_one_wall(self):
        x = self.width/2 + 10
        y1 = self.height/2 - 12
        y2 = self.height/2 + 12
        wall = self.create_wall(x, y1, x, y2)
        self.env.add(wall)

    def create_wall(self, x1, y1, x2, y2):
        env_b = self.env.static_body
        wall_shape = pymunk.Segment(env_b, Vec2d(x1,y1), Vec2d(x2,y2), \
                                    self.wall_thickness)
        wall_shape.filter = ShapeFilter(categories = WALL_MASK)
        return wall_shape

    def create_robots(self):
        for i in range(self.number_robots):
            puck_mask = RED_PUCK_MASK

            robot = Robot()
            offset = int(self.wall_thickness + robot.radius)
            placed = False
            while not placed:
                robot.body.position = self.get_random_pos_within_border(offset)
                robot.body.angle = uniform(-pi, pi)

                # Uncomment for odometry-test
                #robot.body.position = self.width/4, self.height/2
                #robot.body.angle = 0

                if self.env.shape_query(robot.shape) == []:
                    placed = True
            self.env.add(robot.body, robot.shape)

            # Create the robot's sensors
            robot.sensor_suite = SensorSuite(puck_mask)

            # Create the controller
            if self.controller_name == "EchoController":
                robot.controller = EchoController()
#            elif self.controller_name == "GauciImageController":
#                robot.controller = GauciImageController(puck_mask)
            elif self.controller_name == "GauciRangeController":
                robot.controller = GauciRangeController(puck_mask)
#            elif self.controller_name == "OdoBumpController":
#                robot.controller = OdoBumpController(robot, puck_mask)
            elif self.controller_name == "BumpController":
                robot.controller = BumpController(robot, puck_mask)
            elif self.controller_name == "SimpleBumpController":
                robot.controller = SimpleBumpController(robot, puck_mask)
            #elif self.controller_name == "TestOdometerController":
            #    robot.controller = TestOdometerController()

            self.robots.append(robot)

    def create_pucks_random(self):
        for i in range(self.number_pucks):
            puck = Puck(choice(self.puck_kinds), self.puck_shape)
            #offset = int(self.wall_thickness + puck.radius)
            offset = int(puck.radius)
            placed = False
            while not placed:
                puck.body.position = self.get_random_pos_within_border(offset)
                if self.env.shape_query(puck.shape) == []:
                    placed = True
            self.env.add(puck.body, puck.shape)

            self.pucks.append(puck)

    def create_pucks_ring(self):
        centre_x = self.width / 2
        centre_y = self.height / 2
        radius = self.puck_ring_radius

        for i in range(self.number_pucks):
            angle = i / float(self.number_pucks) * 2*pi
            if angle >= pi/4 and angle <= (2*pi - pi/4):
                x = centre_x + radius * cos(angle)
                y = centre_y + radius * sin(angle)
                self.create_puck((x, y))

    def create_immobile_pucks(self):
        x = self.width/6
        y_top = 2*self.height/3 + 20 
        y_bot = self.height/3 - 20

        self.create_puck((x, y_top), True)
        self.create_puck((x, y_bot), True)

    def create_puck(self, pos, immobile=False):
        puck = Puck(choice(self.puck_kinds), self.puck_shape, immobile=immobile)
        puck.body.position = pos
        self.env.add(puck.body, puck.shape)
        self.pucks.append(puck)
        return puck

    def create_landmark(self, pos, mask):
        landmark = Landmark(mask)
        landmark.body.position = pos
        self.env.add(landmark.body, landmark.shape)
        self.landmarks.append(landmark)
        return landmark

    def create_canned_landmarks(self):
        if self.canned_landmarks_name == "ONE_CENTRAL":
            # One central landmark
            x = self.width/2
            y = self.height/2
            self.create_landmark((x, y), POLE_LANDMARK_MASK)

        elif self.canned_landmarks_name == "PAIR":
            x = self.width/2
            y = self.height/2
            dx = self.width/5
            self.create_landmark((x-dx, y), POLE_LANDMARK_MASK)
            self.create_landmark((x+dx, y), POLE_LANDMARK_MASK)

        elif self.canned_landmarks_name == "L_SHAPE":
            x = self.width/2
            y = self.height/2
            dx = self.width/5
            dy = self.height/4
            self.create_landmark((x-dx, y+dy), POLE_LANDMARK_MASK)
            self.create_landmark((x-dx, y-dy), POLE_LANDMARK_MASK)
            self.create_landmark((x+dx, y-dy), POLE_LANDMARK_MASK)

        elif self.canned_landmarks_name == "M_SHAPE":
            x = self.width/2
            y = self.height/2
            dx = self.width/5
            dy = self.height/4
            self.create_landmark((x-1.4*dx, y-dy), POLE_LANDMARK_MASK)
            self.create_landmark((x-1.2*dx, y), POLE_LANDMARK_MASK)
            self.create_landmark((x-dx, y+dy), POLE_LANDMARK_MASK)
            self.create_landmark((x, y+dy/1.5), POLE_LANDMARK_MASK)
            self.create_landmark((x+dx, y+dy), POLE_LANDMARK_MASK)
            self.create_landmark((x+1.2*dx, y), POLE_LANDMARK_MASK)
            self.create_landmark((x+1.4*dx, y-dy), POLE_LANDMARK_MASK)

        elif self.canned_landmarks_name == "T_SHAPE":
            dl = self.lmark_pair_dist
            x = self.width/2
            y = self.height/2 - dl/4
            self.create_landmark((x, y-dl), POLE_LANDMARK_MASK)
            self.create_landmark((x, y), POLE_LANDMARK_MASK)
            self.create_landmark((x, y+dl), POLE_LANDMARK_MASK)
            self.create_landmark((x-dl, y+dl), POLE_LANDMARK_MASK)
            self.create_landmark((x+dl, y+dl), POLE_LANDMARK_MASK)

        elif self.canned_landmarks_name == "C_SHAPE":
            self.create_landmarks_ring(8, self.lmark_pair_dist / (2*pi/8), True)

    def create_landmarks_ring(self, number_landmarks, ring_radius, skip_first=False):
        centre_x = self.width / 2
        centre_y = self.height / 2

        for i in range(number_landmarks):
            if skip_first and i == 0:
                continue

            angle = i / float(number_landmarks) * 2*pi
            x = centre_x + ring_radius * cos(angle)
            y = centre_y + ring_radius * sin(angle)
            self.create_landmark((x, y), POLE_LANDMARK_MASK)

    def create_landmarks_random(self):
        for i in range(self.number_landmarks):
            landmark = Landmark(POLE_LANDMARK_MASK)
            offset = self.wall_thickness + landmark.radius
            placed = False
            while not placed:
                landmark.body.position = self.get_random_pos_within_border(offset)
                if self.env.shape_query(landmark.shape) == []:
                    placed = True
            self.env.add(landmark.body, landmark.shape)
            self.landmarks.append(landmark)

    def create_probe_grid(self):

        positions = []
        delta = 20
        margin = 20
        for x in range(margin, self.width - margin, delta):
            for y in range(margin, self.height - margin, delta):
                positions.append((x, y))

        for pos in positions:
            probe = Probe()
            probe.body.position = pos

            # Create the probe's sensors
            probe.range_scanner = RangeScanner("RangeScan:nonlandmarks", WALL_MASK|ROBOT_MASK|ANY_PUCK_MASK|ANY_LANDMARK_MASK, WALL_MASK|ROBOT_MASK|RED_PUCK_MASK)
            probe.landmark_scanner = RangeScanner("RangeScan:landmarks", WALL_MASK|ANY_LANDMARK_MASK, WALL_MASK|ANY_LANDMARK_MASK)

            self.probes.append(probe)

    #def collision_handler(self, arbiter, space, data):
    #    self.collisions += 1
    #    return True

    def create_scalar_field(self, pos, mask):
        landmark = Landmark(mask)
        landmark.body.position = pos
        self.env.add(landmark.body, landmark.shape)
        self.landmarks.append(landmark)
        return landmark

    def update(self, real_time_since_last_update):
        """
        if self.avg_dt == None:
            self.avg_dt = dt
        else:
            self.avg_dt += dt
            if self.steps % 10 == 0:
                print "avg update dt: {}".format(self.avg_dt / (self.steps + 1))
        """

        for robot in self.robots:
            self.update_for_robot(robot)
            robot.control_step()
            
        dt = 0.25
        sim.env.step(dt)

        if self.analyze and self.steps % self.capture_interval == 0:
            self.analyzer.analyze(self.steps, self.pucks, self.robots)

        self.steps += 1

    def update_for_robot(self, robot):

        # First do autonomous control
        #sensor_dump = robot.sensor_suite.compute(self.env, robot)
        sensor_dump = robot.sensor_suite.compute(self.env, robot, self.landmarks)
        twist = robot.controller.react(robot, sensor_dump, True)

        if not self.allow_translation:
            twist.linear = 0
        if not self.allow_rotation:
            twist.angular = 0

        if twist.linear > MAX_LINEAR_SPEED:
            twist.linear = MAX_LINEAR_SPEED
        if twist.linear < -MAX_LINEAR_SPEED:
            twist.linear = -MAX_LINEAR_SPEED
        if twist.angular > MAX_ANGULAR_SPEED:
            twist.angular = MAX_ANGULAR_SPEED
        if twist.angular < -MAX_ANGULAR_SPEED:
            twist.angular = -MAX_ANGULAR_SPEED

        robot.set_command(twist)

# make module runnable from command line
if __name__ == '__main__':

    n = len(sys.argv)
    config_file = None
    trial_number = 0
    if n == 1:
        config_file = "default.cfg"
    elif n == 2:
        config_file = sys.argv[1]
    elif n == 3:
        config_file = sys.argv[1]
        trial_number = int(sys.argv[2])
    else:
        print("usage:\n\talvinsim [config_file] [trial_number]")
        sys.exit(-1)

    sim = AlvinSim(config_file, trial_number)

    if gui_setting.use:
        win = AlvinWindow(sim)
        win.run()
    else:
        #start_time = time.clock()
        while sim.steps <= sim.number_steps:

            sim.update(0)

            #if sim.steps % 10 == 0:
            #    elapsed = (time.clock() - start_time) / 10
            #    print elapsed
            #    start_time = time.clock()

            # Progress indicator
            if sim.steps % (sim.number_steps/10) == 0:
                percentComplete = (100.0 * sim.steps) / sim.number_steps
                print('{}%, '.format(percentComplete))
