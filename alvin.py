#!/usr/bin/env python

import pyglet, pymunk, pymunk.pyglet_util, copy, os, sys, shutil, time

from math import pi, cos, sin, sqrt
from pyglet.window import key
from pyglet.gl import *
from pymunk import Vec2d, ShapeFilter
from random import seed, randint, choice, uniform
from PIL import Image

from puck import Puck
from landmark import Landmark
from robot import Robot
from probe import Probe
from common import *
from sensors import RangeScan, RangeScanner
from sensorsuite import SensorSuite
from controllers import *
from configsingleton import ConfigSingleton
from analysis import *

class AlvinSim(pyglet.window.Window):

    # UI related
    steps_between_toggles = 10
    visualize_puck_sensor = False
    visualize_landmark_sensor = False
    visualize_controllers = False
    allow_translation = True
    allow_rotation = True
    steps_since_toggle = 0

    # Analysis related
    steps = 0
    start_time = time.clock()
    capture_interval = 10
    #collisions = 0
    #cum_speed = 0

    def __init__(self, config_file, trial_number):
        # Use the base of the config file's name as the name of the output dir
        output_dir_base = str.split(config_file, '.')[0]
        self.output_dir = output_dir_base + '/' + str(trial_number)
        print "OUTPUT DIR: "
        print self.output_dir

        config = ConfigSingleton.get_instance(config_file)

        # Load parameters from config file.  The values 'width' and 'height'
        # will actually be set in the call to 'super' below.
        w = config.getint("AlvinSim", "width")
        h = config.getint("AlvinSim", "height")
        self.circ_border = config.getboolean("AlvinSim", "circ_border")
        self.number_robots = config.getint("AlvinSim", "number_robots")
        self.number_pucks = config.getint("AlvinSim", "number_pucks")
        self.number_puck_kinds = config.getint("AlvinSim", "number_puck_kinds")
        self.number_landmarks = config.getint("AlvinSim", "number_landmarks")
        self.number_steps = config.getint("AlvinSim", "number_steps")
        self.puck_ring = config.getboolean("AlvinSim", "puck_ring")
        self.puck_ring_radius = config.getint("AlvinSim", "puck_ring_radius")
        self.landmark_ring = config.getboolean("AlvinSim", "landmark_ring")
        self.landmark_ring_radius = config.getint("AlvinSim", "landmark_ring_radius")
        self.puck_kinds = range(self.number_puck_kinds)
        self.wall_thickness = config.getint("AlvinSim", "wall_thickness")
        self.analyze = config.getboolean("AlvinSim", "analyze")
        self.capture_screenshots = config.getboolean("AlvinSim", "capture_screenshots")
        self.visualize_probes = config.getboolean("AlvinSim", "visualize_probes")
        self.controller_name = config.get("AlvinSim", "controller_name")

        super(AlvinSim, self).__init__(w, h, visible=False)

        self.set_caption(config_file)

        self.keyboard = key.KeyStateHandler()
        self.push_handlers(self.keyboard)
        self.draw_options = pymunk.pyglet_util.DrawOptions()
        self.draw_options.flags = self.draw_options.DRAW_SHAPES

        # Help text
        """
        self.helpLabel = pyglet.text.Label(
            'ESC: quit, arrow-keys: move, p: puck sens, l: lmark sens, c: vis ctrls, t: trans, r: rot',
            font_size=12,
            x=self.width//2, y=self.height//20,
            anchor_x='center', anchor_y='center')
        """

        # Label to show various stats
        self.statsLabel = pyglet.text.Label(
            font_size=18,
            x=20, y=self.height - 30,
            anchor_x='left', anchor_y='center', multiline=True, width=200)

        self.set_stats_label_text()

        # Objects for mouse interaction
        self.spring_body = None
        self.selected_static_body = None
        self.mouse_body = pymunk.Body(body_type = pymunk.Body.KINEMATIC)

        # build simulation environment
        self.env = pymunk.Space()
        self.env.damping = 0.01 # 99% of velocity is lost per second

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
        #if self.landmark_ring:
        #    self.create_landmarks_ring()
        #else:
        #    self.create_landmarks_random()
        self.create_canned_landmarks()
        if self.visualize_probes:
            self.create_probe_grid()

        # Prep for capturing screenshots
        if self.capture_screenshots:
            shutil.rmtree(self.output_dir, ignore_errors=True)
            os.makedirs(self.output_dir)

        if self.analyze:
            init(self.output_dir)

        # Schedule the key callbacks
        for robot in self.robots:
            pyglet.clock.schedule_interval(robot.control_step, 1.0/120)
        pyglet.clock.schedule_interval(self.update, 1.0/60)
        pyglet.clock.schedule_interval(self.env.step, 1.0/60)

        # Setup to handle collisions
        #self.env.add_default_collision_handler().begin = self.collision_handler

        # start simulation
        self.set_visible(True)

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
        radius = min(cx, cy)
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
            # We vary the mask used to detect pucks for both the range sensor
            # and the controller.
            puck_mask = RED_PUCK_MASK

            robot = Robot()
            offset = int(self.wall_thickness + robot.radius)
            placed = False
            while not placed:
                robot.body.position = self.get_random_pos_within_border(offset)
                if self.env.shape_query(robot.shape) == []:
                    placed = True
            self.env.add(robot.body, robot.shape)

            # Create the robot's sensors
            robot.range_scanner = RangeScanner("RangeScan:nonlandmarks", WALL_MASK|ROBOT_MASK|ANY_PUCK_MASK|ANY_LANDMARK_MASK, WALL_MASK|ROBOT_MASK|puck_mask)
            #robot.landmark_scanner = RangeScanner("RangeScan:landmarks", WALL_MASK|ANY_LANDMARK_MASK, WALL_MASK|LANDMARK_MASK)
            robot.landmark_scanner = RangeScanner("RangeScan:landmarks", WALL_MASK|ANY_LANDMARK_MASK, WALL_MASK|ANY_LANDMARK_MASK)

            # Create the controller
            if self.controller_name == "EchoController":
                robot.controller = EchoController()
            elif self.controller_name == "SimpleAvoidController":
                robot.controller = SimpleAvoiderController()
            elif self.controller_name == "RVOAvoiderController":
                robot.controller = RVOAvoiderController(1)
            elif self.controller_name == "OldGauciController":
                robot.controller = OldGauciController()
            elif self.controller_name == "GauciController":
                robot.controller = GauciController(puck_mask)
            elif self.controller_name == "MyController":
                robot.controller = MyController(puck_mask)
            elif self.controller_name == "LeftmostController":
                robot.controller = LeftmostController(puck_mask)
            elif self.controller_name == "LandmarkWallController":
                robot.controller = LandmarkWallController(puck_mask)
            elif self.controller_name == "PushoutController":
                robot.controller = PushoutController(puck_mask)
            elif self.controller_name == "BounceController":
                robot.controller = BounceController(ANY_LANDMARK_MASK,puck_mask)
            elif self.controller_name == "LandmarkCircleController":
                robot.controller = LandmarkCircleController(robot, puck_mask)
            elif self.controller_name == "FlowController":
                robot.controller = FlowController(robot, puck_mask)
            elif self.controller_name == "OutlierBumpController":
                robot.controller = OutlierBumpController(robot, puck_mask)
            elif self.controller_name == "LineBumpController":
                robot.controller = LineBumpController(robot, puck_mask)

            self.robots.append(robot)

    def create_pucks_random(self):
        for i in range(self.number_pucks):
            puck = Puck(choice(self.puck_kinds))
            offset = int(self.wall_thickness + puck.radius)
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
        puck = Puck(choice(self.puck_kinds), immobile=immobile)
        puck.body.position = pos
        self.env.add(puck.body, puck.shape)
        self.pucks.append(puck)
        return puck

    def create_landmark(self, pos, mask, radius):
        landmark = Landmark(mask, radius)
        landmark.body.position = pos
        self.env.add(landmark.body, landmark.shape)
        self.landmarks.append(landmark)
        return landmark

    def create_canned_landmarks(self):
        # Hockey puck radius = 23mm = 0.023
        #arc_radius = 0.023 * M_TO_PIXELS
        #pole_radius = 0.023 * M_TO_PIXELS
        #arc_radius = 20

        # One central landmark
        #x = self.width/2
        #y = self.height/2
        #self.create_landmark((x, y), ARC_LANDMARK_MASK, arc_radius)

        # Enter and exit arc landmarks to form a 'C'.
        #arc_radius = 10
        #self.create_landmark((x+15, y+10), ENTER_ARC_LANDMARK_MASK, arc_radius)
        #self.create_landmark((x+15, y-10), EXIT_ARC_LANDMARK_MASK, arc_radius)

        # Tower of landmarks
        #x = self.width/2
        #y_list = [self.height/2 + i for i in range(-100, 100, 20)]
        #for y in y_list:
        #    self.create_landmark((x, y), POLE_LANDMARK_MASK, pole_radius)
        #self.create_landmark((x, self.height/2 + 200), ARC_LANDMARK_MASK, arc_radius)

        # Two "competing" landmarks
        #arc_radius = 20
        #y = self.height/2
        #self.create_landmark((self.width/2-100, y), ARC_LANDMARK_MASK, arc_radius)
        #self.create_landmark((self.width/2+100, y), ARC_LANDMARK_MASK, arc_radius)
        # To form "1 5 0"
        #self.create_one_landmarks()
        #self.create_five_landmarks()
        #self.create_zero_landmarks()

        # To form "C"
        #arc_radius = 20
        #blast_radius = 10
        #x = self.width/2
        #y = self.height/2
        #self.create_landmark((x, y), ARC_LANDMARK_MASK, arc_radius)
        #self.create_landmark((x+15, y), BLAST_LANDMARK_MASK, blast_radius)

        # Three vertically arranged poles
        radius = 20
        x = self.width/2
        y = self.height/2
        dx = self.width/5
        dy = self.height/5
        self.create_landmark((x-dx, y+dy), POLE_LANDMARK_MASK, radius)
        self.create_landmark((x+dx, y+dy), POLE_LANDMARK_MASK, radius)
        self.create_landmark((x+dx, y-dy), POLE_LANDMARK_MASK, radius)
        #self.create_landmark((x, y), POLE_LANDMARK_MASK, radius)

    def create_one_landmarks(self):
        x = self.width/6
        y_top = 2*self.height/3 + 80 
        y_bot = self.height/3 - 80

        self.create_landmark((x, y_top), POLE_LANDMARK_MASK, 20)
        self.create_landmark((x, y_bot), POLE_LANDMARK_MASK, 20)


    def create_five_landmarks(self):
        """ Landmarks to create the digit '5' in the centre. """
        arc_radius = 20
        blast_radius = 10

        x = self.width/2
        y_top = 2*self.height/3 - 7
        y_bot = self.height/3 + 7

        self.create_landmark((x, y_top), ARC_LANDMARK_MASK, arc_radius)
        self.create_landmark((x, y_bot), ARC_LANDMARK_MASK, arc_radius)

        self.create_landmark((x+15, y_top), BLAST_LANDMARK_MASK, blast_radius)
        self.create_landmark((x-15, y_bot), BLAST_LANDMARK_MASK, blast_radius)

    def create_zero_landmarks(self):
        """ Landmarks to help form the digit '0' on the right. """
        arc_radius = 20

        # A single landmark to form the zero
        # x = 5*self.width/6
        # y = self.height/2
        # self.create_landmark((x, y), ARC_LANDMARK_MASK, 20)

        x = 5*self.width/6
        y_top = 2*self.height/3 - 7
        y_mid = self.height/2
        y_bot = self.height/3 + 7

        self.create_landmark((x, y_top), ARC_LANDMARK_MASK, arc_radius)
        self.create_landmark((x, y_mid), ARC_LANDMARK_MASK, arc_radius)
        self.create_landmark((x, y_bot), ARC_LANDMARK_MASK, arc_radius)

    def create_landmarks_ring(self):
        centre_x = self.width / 2
        centre_y = self.height / 2
        radius = self.landmark_ring_radius

        for i in range(self.number_landmarks):
            angle = i / float(self.number_landmarks) * 2*pi
            x = centre_x + radius * cos(angle)
            y = centre_y + radius * sin(angle)
            self.create_landmark((x, y))

    def create_landmarks_random(self):
        for i in range(self.number_landmarks):
            landmark = Landmark(ARC_LANDMARK_MASK, 10)
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

    def unschedule(self):
        for robot in self.robots:
            pyglet.clock.unschedule(robot.control_step)
        pyglet.clock.unschedule(self.update)
        pyglet.clock.unschedule(self.env.step)

    def set_stats_label_text(self):
        secs = int(time.clock() - self.start_time)
        hours = secs / 3600
        secs -= hours * 3600
        mins = secs / 60
        secs -= mins * 60
        self.statsLabel.text = \
            "steps: {}\ntime: {}:{}:{}".format(self.steps, hours, mins, secs)

    """
    def collision_handler(self, arbiter, space, data):
        self.collisions += 1
        return True
    """

    def visualize_for_robot(self, robot):
        range_scan = robot.range_scanner.compute(self.env, robot)
        landmark_scan = robot.landmark_scanner.compute(self.env, robot)

        #landmark_scan = thin_scan(robot, landmark_scan, ANY_LANDMARK_MASK)

        if self.visualize_puck_sensor:
            robot.range_scanner.visualize(robot, range_scan)
        if self.visualize_landmark_sensor:
            robot.landmark_scanner.visualize(robot, landmark_scan)

        sensor_suite = SensorSuite(range_scan, landmark_scan)
        # Call the controller's react method, although we will actually
        # ignore the resulting twist here.
        robot.controller.react(robot, sensor_suite, self.visualize_controllers)

    def visualize_for_probe(self, probe):
        landmark_scan = probe.landmark_scanner.compute(self.env, probe)
        if self.visualize_landmark_sensor:
            probe.landmark_scanner.visualize(probe, landmark_scan)
        probe.react(landmark_scan)

    def save_screenshot(self):
        # The file index will increase by one each time as that's more
        # convenient for later turning these images into a video.
        index = self.steps / self.capture_interval
        file_name = '{}/{:09d}.png'.format(self.output_dir, index)
        buffer = pyglet.image.get_buffer_manager().get_color_buffer()
        b_image = buffer.image_data.get_image_data()
        pil_image = Image.frombytes(b_image.format, 
                                    (b_image.width, b_image.height),
                           b_image.get_data(b_image.format, b_image.pitch))
        pil_image = pil_image.transpose(Image.FLIP_TOP_BOTTOM)
        pil_image = pil_image.convert('RGB')
        pil_image.save(file_name, 'PNG')

    # define how to draw the visualization
    def on_draw(self):
        # always clear and redraw for graphics programming
        self.clear()
        self.env.debug_draw(self.draw_options)
        #self.helpLabel.draw()
        self.set_stats_label_text()
        self.statsLabel.draw()
        if (self.visualize_puck_sensor or self.visualize_landmark_sensor or
            self.visualize_controllers):
            for robot in self.robots:
                self.visualize_for_robot(robot)
        if self.visualize_probes:
            for probe in self.probes:
                self.visualize_for_probe(probe)

        for landmark in self.landmarks:
            landmark.visualize_params()

        if self.analyze and self.steps % self.capture_interval == 0:
            analyze_puck_distribution(self.steps, self.pucks)

        if self.capture_screenshots and self.steps % self.capture_interval == 0:
            self.save_screenshot()

    def on_mouse_press(self, x, y, button, modifiers):
        self.mouse_body.position = x,y
        hit = self.env.point_query_nearest((x,y), 10, pymunk.ShapeFilter())
        if hit != None:
            body = hit.shape.body
            if body.body_type == pymunk.Body.DYNAMIC:
                rest_length = self.mouse_body.position.get_distance(\
                                                                body.position)
                stiffness = 500
                damping = 10
                self.spring_body = pymunk.DampedSpring(self.mouse_body, body, \
                                  (0,0), (0,0), rest_length, stiffness, damping)
                self.env.add(self.spring_body)
            elif body.body_type == pymunk.Body.STATIC: # e.g. landmarks
                self.selected_static_body = body
                self.env.remove(body)
                self.env.remove(body.shapes)
                
    def on_mouse_release(self, x, y, button, modifiers):
        if self.spring_body != None:
            self.env.remove(self.spring_body)
            self.spring_body = None
        if self.selected_static_body != None:
            self.selected_static_body.position = (x, y)
            self.env.add(self.selected_static_body)
            self.env.add(self.selected_static_body.shapes)
            self.selected_static_body = None
            

        
    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        self.mouse_body.position = x,y


    def handle_keys(self):
        manual_twist = Twist()

        if self.keyboard[key.RIGHT]:
            manual_twist.angular = -0.25 * MAX_ANGULAR_SPEED
        if self.keyboard[key.LEFT]:
            manual_twist.angular = 0.25 * MAX_ANGULAR_SPEED
        if self.keyboard[key.UP]:
            manual_twist.linear = 0.25 * MAX_LINEAR_SPEED
        if self.keyboard[key.DOWN]:
            manual_twist.linear = -0.25 * MAX_LINEAR_SPEED

        # Handle the keyboard events which shouldn't be toggled too quickly.
        if self.keyboard[key.P] and \
            self.steps_since_toggle > self.steps_between_toggles:

            self.visualize_puck_sensor = not self.visualize_puck_sensor
            self.steps_since_toggle = 0
        if self.keyboard[key.L] and \
            self.steps_since_toggle > self.steps_between_toggles:

            self.visualize_landmark_sensor = not self.visualize_landmark_sensor
            self.steps_since_toggle = 0
        if self.keyboard[key.C] and \
            self.steps_since_toggle > self.steps_between_toggles:

            self.visualize_controllers = not self.visualize_controllers
            self.steps_since_toggle = 0
        if self.keyboard[key.T] and \
            self.steps_since_toggle > self.steps_between_toggles:

            self.allow_translation = not self.allow_translation
            self.steps_since_toggle = 0
        if self.keyboard[key.R] and \
            self.steps_since_toggle > self.steps_between_toggles:

            self.allow_rotation = not self.allow_rotation
            self.steps_since_toggle = 0
        if self.keyboard[key.SPACE] and \
            self.steps_since_toggle > self.steps_between_toggles:

            self.allow_translation = not self.allow_translation
            self.allow_rotation = not self.allow_rotation
            self.steps_since_toggle = 0


        return manual_twist

    def update(self, dt):
        manual_twist = self.handle_keys()

        for robot in self.robots:
            self.update_for_robot(dt, robot, manual_twist)
            #self.cum_speed += robot.body.velocity.get_length()

        #analyze_puck_distribution(self.pucks)

        self.steps_since_toggle += 1
        self.steps += 1

        if self.number_steps != -1 and self.steps > self.number_steps:
            if self.analyze:
                save_plots(self.output_dir)
            self.unschedule()
            self.close()
            pyglet.app.exit()

    def update_for_robot(self, dt, robot, manual_twist):

        # First do autonomous control
        range_scan = robot.range_scanner.compute(self.env, robot)
        landmark_scan = robot.landmark_scanner.compute(self.env, robot)
        #landmark_scan = robot.landmark_scanner.compute(self.env, robot, \
        #                                               self.landmarks, False)
        sensor_suite = SensorSuite(range_scan, landmark_scan)
        controller_twist = robot.controller.react(robot, sensor_suite, False)
        twist = copy.deepcopy(manual_twist)

        # Combine manual and controller twists
        if self.allow_translation:
            twist.linear += controller_twist.linear
        if self.allow_rotation:
            twist.angular += controller_twist.angular

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
#    global SIM_STEPS

# EXPERIMENT VARING 'SIM_STEPS'
#
#    for i in range(1, 20):
#        if i == 0:
#            SIM_STEPS = 1
#        else:
#            SIM_STEPS = i
#        print "SIM_STEPS: {}".format(SIM_STEPS)
#        sim = AlvinSim()

#    SIM_STEPS = 1

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
        print "usage:\n\talvin [config_file] [trial_number]"
        sys.exit(-1)

    sim = AlvinSim(config_file, trial_number)
    pyglet.app.run()
