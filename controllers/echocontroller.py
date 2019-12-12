""" Controller which just prints the robots position and orientation. """

import pyglet
from controllers.controller import Controller
from common import Twist

class EchoController(Controller):

    def react(self, this_robot, sensor_suite, visualize=False):
        twist = Twist()

        #print "x, y, theta: {}, {}, {} ".format(this_robot.body.position.x, this_robot.body.position.y, this_robot.body.angle)

        return twist
