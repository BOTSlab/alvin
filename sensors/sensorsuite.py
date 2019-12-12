from common import *
from sensors.pointsamplecam import PointSampleCam
from sensors.rangescanner import RangeScanner
from sensors.landmarkscanner import LandmarkScanner
from sensors.odometer import Odometer
from sensors.sensordump import SensorDump
from sensors.extract_blobs_closest_points import *

class SensorSuite:
    def __init__(self, puck_mask):
        #self.range_scanner = RangeScanner("RangeScan:nonlandmarks", WALL_MASK|ROBOT_MASK|ANY_PUCK_MASK|ANY_LANDMARK_MASK, WALL_MASK|ROBOT_MASK|puck_mask)
        #self.landmark_scanner = RangeScanner("RangeScan:landmarks", WALL_MASK|ANY_LANDMARK_MASK, WALL_MASK|ANY_LANDMARK_MASK)

        self.lower_cam = PointSampleCam(
            "../puck_calib/interpolated_correspondences.csv",
            WALL_MASK|ROBOT_MASK|puck_mask,
            WALL_MASK|ROBOT_MASK|ANY_PUCK_MASK|ANY_LANDMARK_MASK,
            True)
#        self.upper_cam = PointSampleCam("../puck_calib/interpolated_correspondences.csv", WALL_MASK|ANY_LANDMARK_MASK, WALL_MASK|ANY_LANDMARK_MASK, False)
        self.landmark_scanner = LandmarkScanner()

        self.odometer = Odometer()

    def compute(self, env, robot, landmarks=None):
        #range_scan = self.range_scanner.compute(env, robot)
        landmark_scan = self.landmark_scanner.compute(env, robot, landmarks)

        lower_image = self.lower_cam.compute(env, robot)
        #upper_image_raw = self.upper_cam.compute(env, robot)
        #upper_image = extract_blobs_closest_points(robot, upper_image_raw,
        #                                           ANY_LANDMARK_MASK)
        #if landmarks != None:
        #    landmark_scan = self.landmark_scanner.compute(env, robot, landmarks)

        odo_pose = self.odometer.compute(robot)

        #return SensorDump(range_scan, landmark_scan, odo_pose)
#        return SensorDump(range_scan)
        #return SensorDump(lower_image, upper_image, odo_pose)
        return SensorDump(lower_image, landmark_scan, odo_pose)
#        return SensorDump(lower_image)

    def visualize(self, sensor_dump, env, robot, visualize_puck_sensor, visualize_landmark_sensor, landmarks=None):
        if visualize_puck_sensor:
            self.lower_cam.visualize(robot, sensor_dump.lower_image)
            #self.range_scanner.visualize(robot, sensor_dump.range_scan)
        if visualize_landmark_sensor and landmarks != None:
            #self.upper_cam.visualize(robot, sensor_dump.upper_image)
            #self.landmark_scanner.visualize(robot, sensor_dump.landmark_scan)
            self.landmark_scanner.compute(env, robot, landmarks, True)

        # Vis for odo?
