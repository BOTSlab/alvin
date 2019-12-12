from .types import Twist
from math import pi

#CM_TO_PIXELS = 2.4

#CM_TO_PIXELS = 3.5

# Modified just to fit on the small Macbook Air screen
CM_TO_PIXELS = 2.4

MAX_LINEAR_SPEED = 100.0 * CM_TO_PIXELS
MAX_ANGULAR_SPEED = 1.0 * 2 * pi

# The types of objects as powers of two, used for filtering shapes
WALL_MASK = 1
ROBOT_MASK = 2
ARC_LANDMARK_MASK = 4
ENTER_ARC_LANDMARK_MASK = 8
EXIT_ARC_LANDMARK_MASK = 16
POLE_LANDMARK_MASK = 32
BLAST_LANDMARK_MASK = 64
ANY_LANDMARK_MASK = ARC_LANDMARK_MASK | \
                    ENTER_ARC_LANDMARK_MASK | \
                    EXIT_ARC_LANDMARK_MASK | \
                    POLE_LANDMARK_MASK | \
                    BLAST_LANDMARK_MASK
RED_PUCK_MASK = 128
GREEN_PUCK_MASK = 256
BLUE_PUCK_MASK = 512
ANY_PUCK_MASK = RED_PUCK_MASK | GREEN_PUCK_MASK | BLUE_PUCK_MASK
