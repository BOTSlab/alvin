from pymunk import Body, Circle, Segment, moment_for_circle, moment_for_segment, ShapeFilter, Vec2d
from common import Twist, RED_PUCK_MASK, GREEN_PUCK_MASK, BLUE_PUCK_MASK, CM_TO_PIXELS
from random import randint, uniform
from math import pi

class Puck(object):
    def __init__(self, kind, puck_shape, immobile=False):
        self.immobile = immobile
        # self.mass = 0.1  # 0.1 kg

        # Create a massless body with 0 moment of inertia.  Pymunk will figure
        # out the mass and moment of inertia based on the density (set below)
        # when the body and shape are both added to the space.
        if not immobile:
            self.body = Body(0, 0)
        else:
            self.body = Body(0, 0, Body.STATIC)

        if puck_shape == "CIRCLE":
            # Hockey puck radius = 23mm = 0.023
            self.radius = 2.3 * CM_TO_PIXELS

            # Double hockey puck radius
            #self.radius = 4.6 * CM_TO_PIXELS

            # Arbitrary radius
            #self.radius = 0.07 * M_TO_PIXELS

            # Plate puck radius = 12.8cm = 0.128
            #self.radius = 0.128 * M_TO_PIXELS

            # Objects from Gauci et al paper: 0.05 meter radius, converted to
            # pixels for display
            # self.radius = 12 * CM_TO_PIXELS

            self.shape = Circle(self.body, self.radius)

        elif puck_shape == "RANDOM_CIRCLES":
            self.radius = randint(1, 5) * CM_TO_PIXELS
            self.shape = Circle(self.body, self.radius) 

        elif puck_shape == "RECTANGLE":
            length = 100
            thickness = 5
            a = Vec2d(0, 0)
            b = Vec2d(length, 0)
            self.shape = Segment(self.body, a, b, thickness)
            self.radius = length / 2

        elif puck_shape == "RANDOM_RECTANGLES":
            length = randint(10, 150)
            thickness = randint(4, 10)
            a = Vec2d(0, 0)
            b = Vec2d(length, 0)
            self.shape = Segment(self.body, a, b, thickness)
            self.radius = length / 2


        radiusForDensity = 2.3 * CM_TO_PIXELS # hockey puck radius
        self.shape.density = 0.1 / (pi * radiusForDensity**2)

        self.body.position = 0, 0
        self.body.angle = uniform(-pi, pi)
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0

        self.kind = kind
        if kind == 0:
            self.shape.color = 200, 100, 100
            self.shape.filter = ShapeFilter(categories = RED_PUCK_MASK)
        elif kind == 1:
            self.shape.color = 100, 200, 100
            self.shape.filter = ShapeFilter(categories = GREEN_PUCK_MASK)
        elif kind == 2:
            self.shape.color = 100, 100, 200
            self.shape.filter = ShapeFilter(categories = BLUE_PUCK_MASK)
        else:
            sys.exit("Unknown puck kind: " + kind)

        if immobile:
            self.shape.color = self.shape.color[0] / 3, self.shape.color[1] / 3, self.shape.color[2] / 3
