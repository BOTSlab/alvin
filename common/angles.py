from math import fabs, floor, pi, atan2, cos, sin

def normalize_angle_0_2pi(theta):
    """Convert angle into positive value in [0,2pi)

    :param float theta: angle
    """
    TWO_PI = 2 * pi
    return theta - TWO_PI * floor(theta / TWO_PI)

def normalize_angle_pm_pi(theta):
    """Convert angle into range [-pi,pi)

    :param float theta: angle
    """
    TWO_PI = 2 * pi
    while theta >= pi:
        theta -= TWO_PI
    while theta < -pi:
        theta += TWO_PI
    return theta

def get_smallest_angular_difference(a, b):
    """ Return smallest distance between two angles.  Result always postive."""
    a = normalize_angle_pm_pi(a)
    b = normalize_angle_pm_pi(b)
    error = fabs(a - b)
    if error > pi:
        error = fabs(error - 2*pi)
    return error

def get_smallest_signed_angular_difference(a, b):
    """ Return angle between the two given angles with the smallest absolute
        value.  Meanwhile, the value returned will have a sign. """
    """
    a = normalize_angle_pm_pi(a)
    b = normalize_angle_pm_pi(b)
    diff1 = a - b
    diff2 = pi - a + (b - pi)
    if fabs(diff1) < fabs(diff2):
        return diff1
    else:
        return -diff2
    """
    # From: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    return atan2(sin(a-b), cos(a-b))

def get_angular_difference(a, b):
    a = normalize_angle_0_2pi(a)
    b = normalize_angle_0_2pi(b)
    error = a - b
    if error < 0:
        error = error + 2*pi
    return error
