from math import cos, sin, pi
from pymunk import Body, Circle, Poly, moment_for_circle, ShapeFilter
from common import Twist, ROBOT_MASK, M_TO_PIXELS
from common.angles import normalize_angle_0_2pi

class Robot(object):
    def __init__(self):
        self.mass = 1  # 1 kg

        # 0.1 meter radius, converted to pixels for display
        #self.radius = 0.05 * M_TO_PIXELS

        # Bupimo: 0.111 meter radius, converted to pixels for display
        self.radius = 0.111 * M_TO_PIXELS

        # moment of inertia for disk
        rob_I = moment_for_circle(self.mass, 0, self.radius)

        self.body = Body(self.mass, rob_I)
        self.body.position = 0, 0
        self.body.angle = 0
        self.body.velocity = 0, 0
        self.body.angular_velocity = 0

        """
        self.shape = Circle(self.body, self.radius)
        self.shape.color = 127, 0, 255  # a pinkish blue
        self.shape.filter = ShapeFilter(categories = ROBOT_MASK)
        """

        """
        r = self.radius
        p = self.radius / 2.0 # Amount the wedge part pokes out.
        vertices = [(r+p, r),
                    (-r/3, r),
                    (-r, 0),
                    (-r/3, -r),
                    (r/3, -r) ]
        """
        r = self.radius
        d = self.radius * 1.5 # Amount the wedge part pokes out.
        vertices = [(0, -r),
                    (d, 0),
                    (0, r)]
        # Now add the semicircular back part
        n = 3
        angles = [pi/2 + i*(pi/n) for i in range(1, n)]
        for a in angles:
            vertices.append((r*cos(a), r*sin(a)))

        self.shape = Poly(self.body, vertices)
        self.shape.color = 127, 0, 255  # a pinkish blue
        self.shape.filter = ShapeFilter(categories = ROBOT_MASK)

        self.command = Twist()

    def control_step(self, dt):
        """Execute one control step for robot.

        control_step should be called regularly and at high frequency
        to ensure propper execution.

        :param dt: time since last control step execution
        :type dt: float
        """

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
