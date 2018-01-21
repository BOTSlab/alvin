""" A PointSampleCam emulates a camera which has been calibrated to associate real-world coordinates (xr, yr) with each pixel position (xp, yp).  A calibration data file is consulted which provides these associations. """

import pymunk
import numpy as np
from math import atan2, sqrt, fabs
from common import *
from pymunk import ShapeFilter
from configsingleton import ConfigSingleton

# Our mechanism for selectively importing pyglet/GUI-related stuff.
import gui_setting
if gui_setting.use:
    import pyglet

class PointSampleImage:
    def __init__(self, calib_array, neighbour_array):
        self.calib_array = calib_array # How costly is this?

        self.neighbour_array = neighbour_array

        # A list of masks, where each entry in the list corresponds to a row of
        # calib_array.
        self.n_rows = self.calib_array.shape[0]
        self.masks = [0] * self.n_rows

class PointSampleCam:
    def __init__(self, calib_filename, detection_mask, acceptance_mask, frontal_only):
        config = ConfigSingleton.get_instance()
        self.min_distance = config.getfloat("PointSampleCam", "min_distance")
        self.max_distance = config.getfloat("PointSampleCam", "max_distance")
        self.max_abs_angle = config.getfloat("PointSampleCam", "max_abs_angle")

        # The detection mask is used to indicate all types of objects that
        # the sensor should be sensitive to.  However, if a detected object
        # doesn't also match the acceptance mask then it will be treated as
        # a wall.
        self.detection_mask = detection_mask
        self.acceptance_mask = acceptance_mask

        self.calib_array = np.loadtxt(calib_filename, delimiter=',')
        self.calib_array[:,2] *= CM_TO_PIXELS
        self.calib_array[:,3] *= CM_TO_PIXELS

        # We will also store within calib_array the following additional
        # quantities derived from (xr, yr) so that we don't need to compute
        # these again.
        #   angle of (xr, yr) w.r.t. to X_R axis --- atan2(yr, xr)
        #   length of (xr, yr)                   --- sqrt(xr*xr + yr*yr)
        n_rows = self.calib_array.shape[0]
        # Add the two extra columns
        self.calib_array = np.append(self.calib_array, np.zeros((n_rows, 2)), axis=1)
        for i in range(n_rows):
            (xr, yr) = self.calib_array[i,2], self.calib_array[i,3]
            self.calib_array[i,4] = atan2(yr, xr)
            self.calib_array[i,5] = sqrt(xr*xr + yr*yr)

        if frontal_only:
            # Delete all rows with distance outside of [min_distance,
            # max_distance] and angle outside of [-max_abs_angle, max_abs_angle]
            delete_indices = []
            for i in range(n_rows):
                angle = self.calib_array[i,4]
                dist = self.calib_array[i,5]
                if fabs(dist) < self.min_distance:
                    delete_indices.append(i)

                if fabs(dist) > self.max_distance:
                    delete_indices.append(i)

                if fabs(angle) > self.max_abs_angle:
                    delete_indices.append(i)

            self.calib_array = np.delete(self.calib_array, delete_indices, axis=0)
        
        # We also pre-compute the indices of the neighbours for each pixel.
        self.neighbour_array = []
        n_rows = self.calib_array.shape[0]
        for i in range(n_rows):
            #(ixi, iyi) = self.calib_array[i,0], self.calib_array[i,1]
            (ixr, iyr) = self.calib_array[i,2], self.calib_array[i,3]
            nghbrs = []
            for j in range(i+1, n_rows):
                #(jxi, jyi) = self.calib_array[j,0], self.calib_array[j,1]
                (jxr, jyr) = self.calib_array[j,2], self.calib_array[j,3]

                """ Determining neighbourhood based on 8-adjacency
                dx = ixi - jxi
                dy = iyi - jyi
                ij_dist = sqrt(dx*dx + dy*dy)
                if ij_dist <= sqrt(2) + 0.01:
                    nghbrs.append(j)
                """

                # Determining neighbourhood based on a threshold distance
                dx = ixr - jxr
                dy = iyr - jyr
                ij_dist = sqrt(dx*dx + dy*dy)
                if ij_dist <= 50:
                    nghbrs.append(j)

            self.neighbour_array.append(nghbrs)
            
        self.shape_filter = ShapeFilter(mask=self.detection_mask)

    def compute(self, env, robot):

        image = PointSampleImage(self.calib_array, self.neighbour_array)

        n_rows = self.calib_array.shape[0]
        for i in range(n_rows):
            # Coordinates of sensed point in robot ref. frame
            (xr, yr) = self.calib_array[i,2], self.calib_array[i,3]

            # Coordinates in world coordinates
            (xw, yw) = robot.body.local_to_world((xr, yr))

            query_info = env.point_query_nearest((xw, yw), 0, self.shape_filter)
            if query_info != None:
                object_mask = query_info.shape.filter.categories
                if object_mask & self.acceptance_mask == 0:
                    # The detected shape is not accepted, we will treat
                    # it as a wall.
                    object_mask = WALL_MASK

                image.masks[i] = object_mask

        return image

    def visualize(self, robot, image):
        n_rows = self.calib_array.shape[0]
        for i in range(n_rows):
            # Coordinates of sensed point in robot ref. frame
            (xr, yr) = self.calib_array[i,2], self.calib_array[i,3]

            # Coordinates in world coordinates
            (xw, yw) = robot.body.local_to_world((xr, yr))

            pyglet.gl.glPointSize(3);
            if image.masks[i] == 0:
                color = (255, 255, 255)
            elif image.masks[i] == WALL_MASK:
                color = (255, 255, 0)
            elif image.masks[i] == ROBOT_MASK:
                color = (0, 255, 255)
            elif image.masks[i] == BLAST_LANDMARK_MASK:
                color = (0, 0, 255)
            elif image.masks[i] == POLE_LANDMARK_MASK:
                color = (255, 0, 0)
            elif image.masks[i] == ARC_LANDMARK_MASK:
                color = (0, 255, 0)
            elif image.masks[i] == RED_PUCK_MASK:
                color = (255, 0, 255)
            elif image.masks[i] == GREEN_PUCK_MASK:
                color = (0, 255, 255)
            elif image.masks[i] == BLUE_PUCK_MASK:
                color = (255, 255, 0)
            else:
                print "Unknown mask: {}".format(image.masks[i])
            pyglet.graphics.draw(1, pyglet.gl.GL_POINTS,
                ('v2f', (xw, yw)), ('c3B', color))

        pyglet.gl.glPointSize(1);

"""
# make module runnable from command line
if __name__ == '__main__':
    print "RUN"
    sampler = PointSampler("../data/phase1_160x120.csv", 0, 0)
    sampler.compute(None, None)

"""
