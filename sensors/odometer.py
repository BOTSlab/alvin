
import pymunk
from math import pi, cos, sin, sqrt, atan2, log
from random import random, gauss
from common import *

from configsingleton import ConfigSingleton

class Odometer:
    def __init__(self):

        config = ConfigSingleton.get_instance()
        self.cheating = config.getboolean("Odometer", "cheating")

        # Parameters.  These are the four alpha parameter's from Thrun et. al's
        # odometry motion model.  We simplify this model so that angular error
        # is represented with ALPHA1 and translational error with ALPHA2.  We
        # further simplify by relating these two parameters linearly, reducing
        # the error model down to one parameter.
        self.alpha = config.getfloat("Odometer", "alpha") / CM_TO_PIXELS
        self.alpha1 = self.alpha * 0.01
        self.alpha2 = self.alpha * 0.1
        self.alpha3 = self.alpha2
        self.alpha4 = 0

        self.x = 0
        self.y = 0
        self.theta = 0
        self.lastX = 0
        self.lastY = 0
        self.lastTheta = 0

        # Needed by noisyEstimate
        self.lastTrueX = None
        self.lastTrueY = None
        self.lastTrueTheta = None

    def internal_rand_normal(self):
        """Approximate sampling from the standard normal distribution
        -- From http://osa1.net/posts/2012-12-19-different-distributions-from-uniform.html
        """
        return sqrt(-2 * log(random())) * cos(2 * pi * random()) / 2
        #return gauss(0, 1)

    def cheat(self, robot):
        """ "Cheating" odometry model which just uses the true robot pose. """

        self.x = robot.body.position.x
        self.y = robot.body.position.y
        self.theta = robot.body.angle

    def compute(self, robot, reset=False):

        if reset:
            self.x = 0
            self.y = 0
            self.theta = 0
            self.lastX = 0
            self.lastY = 0
            self.lastTheta = 0
        
        if self.cheating:
            self.cheat(robot)
        else:
            self.thrunMotionModel(robot)
        
        self.lastX = self.x
        self.lastY = self.y
        self.lastTheta = self.theta

        return (self.x, self.y, self.theta)

    def thrunMotionModel(self, robot):
        """ Use the odometry motion model from Thrun et al's "Probabilistic
        Robotics", chapter 5. """

        # Get the true pose
        trueX = robot.body.position.x
        trueY = robot.body.position.y
        trueTheta = robot.body.angle

        if self.lastTrueX == None:
            self.lastTrueX = trueX
            self.lastTrueY = trueY
            self.lastTrueTheta = trueTheta

        # Determine the three motion parameters
        trans = sqrt((trueX - self.lastTrueX)**2 + (trueY - self.lastTrueY)**2)
        rot1 = atan2(trueY - self.lastTrueY, trueX - self.lastTrueX) - self.lastTrueTheta
        rot2 = trueTheta - self.lastTrueTheta - rot1
        #print "CLEAN trans: {} rot1: {} rot2: {}".format(trans, rot1, rot2)

        # Figure out the variance of the noise processes (why? because bigger
        # motions are noisier).
        transSqd = trans**2
        rot1Sqd = rot1**2
        rot2Sqd = rot2**2
        var_rot1 = self.alpha1 * rot1Sqd + self.alpha2 * transSqd
        var_trans = self.alpha3 * transSqd + self.alpha4 * (rot1Sqd + rot2Sqd)
        var_rot2 = self.alpha1 * rot2Sqd + self.alpha2 * transSqd
        #print "VARIANCES trans: {} rot1: {} rot2: {}".format(var_trans, var_rot1, var_rot2)

        # Now add noise
        #trans = trans + var_trans * self.internal_rand_normal()
        #rot1 = rot1 + var_rot1 * self.internal_rand_normal()
        #rot2 = rot2 + var_rot2 * self.internal_rand_normal()
        trans = gauss(trans, sqrt(var_trans))
        rot1 = gauss(rot1, sqrt(var_rot1))
        rot2 = gauss(rot2, sqrt(var_rot2))
        #print "NOISE trans: {} rot1: {} rot2: {}".format(trans, rot1, rot2)

        self.x = self.lastX + trans * cos(self.lastTheta + rot1)
        self.y = self.lastY + trans * sin(self.lastTheta + rot1)
        self.theta = self.lastTheta + rot1 + rot2
        #print "EST  x: {} y: {} theta: {}".format(self.x, self.y, self.theta)

        self.lastTrueX = trueX
        self.lastTrueY = trueY
        self.lastTrueTheta = trueTheta
