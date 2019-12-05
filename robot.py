from math import cos, sin, pi
from pymunk import Body, Circle, Poly, moment_for_circle, moment_for_poly, \
                   ShapeFilter
from common import Twist, ROBOT_MASK, CM_TO_PIXELS
from common.angles import normalize_angle_0_2pi

class Robot(object):
    def __init__(self):
        self.mass = 1  # 1 kg

        # e-puck: 0.037 meter radius, converted to pixels for display
        #self.radius = 3.7 * CM_TO_PIXELS

        # Bupimo: 9 cm radius, converted to pixels for display
        self.radius = 9 * CM_TO_PIXELS

        self.circular = False;

        if self.circular:
            rob_I = moment_for_circle(self.mass, 0, self.radius)
            self.body = Body(self.mass, rob_I)
            self.shape = Circle(self.body, self.radius)
        else:
            r = self.radius
            d = self.radius * 1.5 # Amount the wedge part pokes out.
            vertices = [(0, -r),
                        (d, 0),
                        (0, r)]
            #vertices = [(0, -r),
            #            (d/4, 0),
            #            (d/2, r)]
            # Now add the semicircular back part
            n = 5
            angles = [pi/2 + i*(pi/n) for i in range(1, n)]
            for a in angles:
                vertices.append((r*cos(a), r*sin(a)))

            rob_I = moment_for_poly(self.mass, vertices)
            self.body = Body(self.mass, rob_I)
            self.shape = Poly(self.body, vertices)

            # EXPERIMENTAL: Add bristles to robot
#            rest_length = 100
#            stiffness = 500
#            damping = 10
#            self.bristle_body = pymunk.DampedSpring(self.body, body, \
#                                  (0,0), (0,0), rest_length, stiffness, damping)
#                self.sim.env.add(self.spring_body)

        self.body.position = 0, 0
        self.body.angle = 0
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0
        self.shape.color = 0, 255, 0
        self.shape.filter = ShapeFilter(categories = ROBOT_MASK)

        self.command = Twist()

        #self.avg_dt = None
        #self.steps = 0

    def control_step(self):
        """Execute one control step for robot."""

        self.body.angular_velocity = self.command.angular
        self.body.apply_impulse_at_local_point((self.command.linear, 0), (0,0))

    def set_command(self, twist):
        """Set robot velocity command.

        :param twist: command to send
        :type twist: roboticsintro.common.Twist
        """
        if twist is None:
            raise ValueError("Command may not be null. Set zero velocities instead.")
        self.command = twist

    def stop(self):
        """Stop robot motion."""
        self.set_command(Twist())

    def get_pose(self):
        return (self.body.position.x,
                self.body.position.y,
                normalize_angle_0_2pi(self.body.angle))
