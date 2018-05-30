import math

from gyroLidar.gydar import *


class DetectionFields(object):
    amount_of_fields = 7
    degrees_front = 0
    fov = 180
    warning_degrees = 30

    _gydar = None
    _gydar_thread = None

    lidar_data_is_trusted = True
    latest_gyro = [None, None]
    latest_bottom = None
    latest_output = [None for _ in range(amount_of_fields)]
    thread_is_active = False

    def __init__(self, lidar_port, gyro_port):
        self._gydar = Gydar()
        self._gydar.set_lidar_port(lidar_port)
        self._gydar.set_gyro_port(gyro_port)

    def start_updating(self):
        self.thread_is_active = True

        self._gydar.connect()

        self._gydar_thread = Thread(target=gydar_loop, args=(self._gydar, self))
        self._gydar_thread.start()

    def close(self):
        self.thread_is_active = False
        self._gydar.disconnect_gyro()
        self._gydar.disconnect_lidar()
        self._gydar_thread.join()


def gydar_loop(gydar: Gydar, fields: DetectionFields):
    middle = fields.degrees_front

    start = round(middle - fields.fov / 2)
    end = round(middle + fields.fov / 2)

    data_length = len(gydar.raw_lidar_output)

    if start < 0:
        start = data_length + start

    if end > data_length:
        end = end % data_length

    while fields.thread_is_active:
        """ If some sensors or USB connections got fucked """
        if gydar.connected != ConnectionStates.BOTH_CONNECTED:
            if gydar.connected == ConnectionStates.GYRO_CONNECTED:
                gydar.disconnect_gyro()
            elif gydar.connected == ConnectionStates.LIDAR_CONNECTED:
                gydar.disconnect_lidar()

            fields.thread_is_active = False

        """ Get data """

        ping_data, gyro_data = gydar.raw_gyro_output
        lidar_data = gydar.raw_lidar_output

        """ Check if degrees are greater than e.g. 30 degrees, then we shouldn't trust the lidar. """

        is_trusted = True
        if gyro_data is not None:
            for deg in gyro_data:
                if deg > fields.warning_degrees:
                    is_trusted = False

        fields.lidar_data_is_trusted = is_trusted

        """ Analyse pingsensor data"""

        front = left = right = 1e10
        try:
            front = ping_data[0] * 10  # 0 degrees front
            left = ping_data[1] * 10  # 45 degrees left
            right = ping_data[2] * 10  # 45 degrees right
            fields.latest_down = ping_data[3] * 10  # pointing down
        except TypeError:
            pass

        degree_per_field = fields.fov / fields.amount_of_fields

        left_field = math.floor((fields.fov / 2 - 45) / degree_per_field)
        right_field = fields.amount_of_fields - left_field
        middle_field = fields.amount_of_fields

        """ Update fields """

        if start > end:
            lidar_data = lidar_data[start:] + lidar_data[:end]
        else:
            lidar_data = lidar_data[start:end]

        lidar_data = chunk_list(lidar_data, fields.amount_of_fields)

        index = 0
        for field in lidar_data:
            try:
                field.sort()
                average = sum(field) / len(field)
                closest = field[0]

                if index == middle_field:
                    if middle < closest:
                        closest = front
                elif index == left_field:
                    if left < closest:
                        closest = left
                elif index == right_field:
                    if right < closest:
                        closest = right

                fields.latest_output[index] = (average, closest, index * degree_per_field)
            except TypeError:
                pass
            index += 1


def chunk_list(seq, num):
    avg = len(seq) / float(num)
    out = []
    last = 0.0

    while last < len(seq):
        out.append(seq[int(last):int(last + avg)])
        last += avg

    return out
