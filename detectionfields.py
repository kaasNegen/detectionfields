from gyroLidar.gydar import *


class DetectionFields(object):
    amount_of_fields = 7
    degrees_front = 0
    fov = 180

    _gydar = None
    _gydar_thread = None

    latest_output = [None for _ in range(amount_of_fields)]
    thread_is_active = False

    def __init__(self):
        self._gydar = Gydar()

    def start_updating(self):
        self.thread_is_active = True
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
        # for the time being no ping sensor data in there.
        gyro_data = gydar.raw_gyro_output

        lidar_data = gydar.raw_lidar_output

        if start > end:
            lidar_data = lidar_data[start:] + lidar_data[:end]
        else:
            lidar_data = lidar_data[start:end]

        lidar_data = chunk_list(lidar_data, fields.amount_of_fields)

        index = 0
        for field in lidar_data:
            field.sort(reverse=True)
            average = sum(field) / len(field)
            closest = field[0]

            fields.latest_output[index] = (average, closest)

            index += 1


def chunk_list(seq, num):
    avg = len(seq) / float(num)
    out = []
    last = 0.0

    while last < len(seq):
        out.append(seq[int(last):int(last + avg)])
        last += avg

    return out
