import pyglet, pymunk.pyglet_util
import time
from pyglet.window import key
from pyglet.gl import *
# from PIL import Image

from robot import Robot
from probe import Probe
from common import *
from sensors import RangeScan, RangeScanner, SensorSuite, SensorDump
from configsingleton import ConfigSingleton
from common.drawing import draw_segment_wrt_robot

class AlvinWindow(pyglet.window.Window):

    # UI related
    steps_between_toggles = 10
    visualize_puck_sensor = False
    visualize_landmark_sensor = False
    visualize_controllers = False
    steps_since_toggle = 0

    def __init__(self, sim):

        super(AlvinWindow, self).__init__(sim.width, sim.height, visible=False)

        self.sim = sim

        self.start_time = time.clock()

        #self.set_caption(config_file)

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
            x=20, y=self.height - 40,
            anchor_x='left', anchor_y='center', multiline=True, width=200)

        self.set_stats_label_text()

        # Objects for mouse interaction
        self.spring_body = None
        self.selected_static_body = None
        self.mouse_body = pymunk.Body(body_type = pymunk.Body.KINEMATIC)

        # Schedule the key callbacks
        for robot in sim.robots:
            pyglet.clock.schedule_interval(robot.control_step, 1.0/120)
        pyglet.clock.schedule_interval(sim.update, 1.0/60)
        pyglet.clock.schedule_interval(sim.env.step, 1.0/60)
        pyglet.clock.schedule_interval(self.handle_keys, 1.0/60)

        self.set_visible(True)

    def unschedule(self):
        for robot in self.sim.robots:
            pyglet.clock.unschedule(robot.control_step)
        pyglet.clock.unschedule(self.sim.update)
        pyglet.clock.unschedule(self.sim.env.step)

    def set_stats_label_text(self):
#        secs = int(time.clock() - self.start_time)
#        hours = secs / 3600
#        secs -= hours * 3600
#        mins = secs / 60
#        secs -= mins * 60
#        self.statsLabel.text = \
#            "steps: {}\ntime: {}:{}:{}".format(self.sim.steps, hours, mins, secs)
        self.statsLabel.text = "steps: {}".format(self.sim.steps)

    def visualize_for_robot(self, robot, landmarks):
        sensor_dump = robot.sensor_suite.compute(self.sim.env, robot, landmarks)

        #robot.sensor_suite.visualize(sensor_dump, robot, self.visualize_puck_sensor, self.visualize_landmark_sensor)
        robot.sensor_suite.visualize(sensor_dump, self.sim.env, robot, self.sim.landmarks, self.visualize_puck_sensor, self.visualize_landmark_sensor)

        # Call the controller's react method, although we will actually
        # ignore the resulting twist here.
        robot.controller.react(robot, sensor_dump, self.visualize_controllers)

    def visualize_for_probe(self, probe):
        landmark_scan = probe.landmark_scanner.compute(self.sim.env, probe)
        if self.visualize_landmark_sensor:
            probe.landmark_scanner.visualize(probe, landmark_scan)
        probe.react(landmark_scan)

    def save_screenshot(self):
        # The file index will increase by one each time as that's more
        # convenient for later turning these images into a video.
        index = self.sim.steps / self.sim.capture_interval
        file_name = '{}/{:07d}.png'.format(self.sim.output_dir, index)
        print "Capturing screenshot to {}".format(file_name)
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
        self.sim.env.debug_draw(self.draw_options)
        #self.helpLabel.draw()
        self.set_stats_label_text()
        self.statsLabel.draw()

        # Draw orientations for all robots.
        for robot in self.sim.robots:
            draw_segment_wrt_robot(robot, (0, 0), (12, 0), (255,255,255), 3)

        if (self.visualize_puck_sensor or self.visualize_landmark_sensor or
            self.visualize_controllers):
            # for robot in self.sim.robots:
            #     self.visualize_for_robot(robot, self.sim.landmarks)
            self.visualize_for_robot(self.sim.robots[0], self.sim.landmarks)

        if self.sim.visualize_probes:
            for probe in self.sim.probes:
                self.visualize_for_probe(probe)

        for landmark in self.sim.landmarks:
            landmark.visualize_params()

        self.set_caption('step: {}'.format(self.sim.steps))

        if self.sim.capture_screenshots and self.sim.steps % self.sim.capture_interval == 0:
            self.save_screenshot()

        if self.sim.number_steps != -1 and self.sim.steps > self.sim.number_steps:
            self.unschedule()
            self.close()
            pyglet.app.exit()
        
    def on_mouse_press(self, x, y, button, modifiers):
        self.mouse_body.position = x,y
        hit = self.sim.env.point_query_nearest((x,y), 10, pymunk.ShapeFilter())
        if hit != None:
            body = hit.shape.body
            if body.body_type == pymunk.Body.DYNAMIC:
                rest_length = self.mouse_body.position.get_distance(\
                                                                body.position)
                stiffness = 500
                damping = 10
                self.spring_body = pymunk.DampedSpring(self.mouse_body, body, \
                                  (0,0), (0,0), rest_length, stiffness, damping)
                self.sim.env.add(self.spring_body)
            elif body.body_type == pymunk.Body.STATIC: # e.g. landmarks
                self.selected_static_body = body
                self.sim.env.remove(body)
                self.sim.env.remove(body.shapes)
                
    def on_mouse_release(self, x, y, button, modifiers):
        if self.spring_body != None:
            self.sim.env.remove(self.spring_body)
            self.spring_body = None
        if self.selected_static_body != None:
            self.selected_static_body.position = (x, y)
            self.sim.env.add(self.selected_static_body)
            self.sim.env.add(self.selected_static_body.shapes)
            self.selected_static_body = None
        
    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        self.mouse_body.position = x,y

    def handle_keys(self, dt):
        manual_twist = Twist()

        if self.keyboard[key.RIGHT]:
            manual_twist.angular = -0.005 * MAX_ANGULAR_SPEED
        if self.keyboard[key.LEFT]:
            manual_twist.angular = 0.005 * MAX_ANGULAR_SPEED
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

            self.sim.allow_translation = not self.sim.allow_translation
            self.steps_since_toggle = 0
        if self.keyboard[key.R] and \
            self.steps_since_toggle > self.steps_between_toggles:

            self.sim.allow_rotation = not self.sim.allow_rotation
            self.steps_since_toggle = 0
        if self.keyboard[key.SPACE] and \
            self.steps_since_toggle > self.steps_between_toggles:

            self.sim.allow_translation = not self.sim.allow_translation
            self.sim.allow_rotation = not self.sim.allow_rotation
            self.steps_since_toggle = 0

        self.steps_since_toggle += 1

        for robot in self.sim.robots:
            robot.body.angle += manual_twist.angular
            robot.body.apply_impulse_at_local_point((manual_twist.linear, 0),
                                                    (0,0))

    def run(self):
        pyglet.app.run()
