from gyroLidar.gydar import *


class DetectionFields(object):
    fields = 6
    degrees_front = 0
    fov = 180

    _gydar = None

    latest_output = None

    def __init__(self):
        self._gydar = Gydar()
