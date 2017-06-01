from pymunk import Body, Circle, moment_for_circle, ShapeFilter
from common import Twist, RED_PUCK_MASK, GREEN_PUCK_MASK, BLUE_PUCK_MASK, M_TO_PIXELS

class Puck(object):
    def __init__(self, kind, immobile=False):
        self.immobile = immobile
        self.mass = 0.1  # 0.1 kg

        # 0.1 meter radius, converted to pixels for display
        #self.radius = 0.1 * M_TO_PIXELS

        # Hockey puck radius = 23mm = 0.023
        self.radius = 0.023 * M_TO_PIXELS

        # Double hockey puck radius
        #self.radius = 0.046 * M_TO_PIXELS

        # Plate puck radius = 12.8cm = 0.128
        #self.radius = 0.128 * M_TO_PIXELS

        # moment of inertia for disk
        moment_inertia = moment_for_circle(self.mass, 0, self.radius)

        if not immobile:
            self.body = Body(self.mass, moment_inertia)
        else:
            self.body = Body(0, 0, Body.STATIC)

        self.body.position = 0, 0
        self.body.angle = 0
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0

        self.shape = Circle(self.body, self.radius)

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
