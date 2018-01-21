class SensorDump:
#    def __init__(self, range_scan, landmark_scan, odo_pose):
#        self.range_scan = range_scan
#        self.landmark_scan = landmark_scan
#    def __init__(self, range_scan):
#        self.range_scan = range_scan

#    def __init__(self, lower_image, upper_image, odo_pose):
    def __init__(self, lower_image, landmarks, odo_pose):
        self.lower_image = lower_image
#        self.upper_image = upper_image
        self.landmarks = landmarks
        self.odo_pose = odo_pose
