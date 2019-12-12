import pyglet
from math import *
from pyglet.gl import *

def draw_ray(robot, angle, color, length=10000, width=6):
    c = cos(robot.body.angle + angle)
    s = sin(robot.body.angle + angle)
    x1 = int(robot.body.position.x)
    y1 = int(robot.body.position.y)
    x2 = int(robot.body.position.x + length * c)
    y2 = int(robot.body.position.y + length * s)

    pyglet.gl.glLineWidth(width)
    pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2f', (x1, y1, x2, y2)),
                                                ('c3B', color+color))
    pyglet.gl.glLineWidth(1)

def draw_segment_wrt_robot(robot, a, b, color=(255,255,255), width=3):
    """ Given points a and b, draw a line segment between them.  These two points are specified in the robot's reference frame. """

    (x1, y1) = robot.body.local_to_world(a)
    (x2, y2) = robot.body.local_to_world(b)
    
    pyglet.gl.glLineWidth(width)
    pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2f', (x1, y1, x2, y2)),
                                                ('c3B', color+color))
    pyglet.gl.glLineWidth(1)


def draw_circle(xxx_todo_changeme, radius, color):
    # Arbitrarily set for now
    (cx, cy) = xxx_todo_changeme
    numPoints = 50

    glColor3f(color[0], color[1], color[2])

    verts = []
    for i in range(numPoints):
        angle = radians(float(i)/numPoints * 360.0)
        x = cx + radius*cos(angle)
        y = cy + radius*sin(angle)
        verts += [x,y]
    circle = pyglet.graphics.vertex_list(numPoints, ('v2f', verts))
    circle.draw(GL_LINE_LOOP)
