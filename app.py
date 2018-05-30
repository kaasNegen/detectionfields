from detectionfields import *
from Haptic.HapticHandler import *

import sys
import glob

import time

gyro_port = lidar_port = haptic_port = None


def find_serial_ports():
    global gyro_port, lidar_port, haptic_port
    old = serial_ports()
    input('Connect Gyroscope and press ENTER')

    new = serial_ports()
    diff = list(set(new) - set(old))
    if len(diff) < 1:
        print('No (new) USB/COM ports found! Exiting!')
        exit()
    else:
        gyro_port = diff[0]
        print('Found', diff[0])
        old = new

    input('Connect Lidar and press ENTER')
    new = serial_ports()
    diff = list(set(new) - set(old))
    if len(diff) < 1:
        print('No (new) USB/COM ports found! Exiting!')
        exit()
    else:
        lidar_port = diff[0]
        print('Found', diff[0])
        old = new

    input('Connect Haptic and press ENTER')
    new = serial_ports()
    diff = list(set(new) - set(old))
    if len(diff) < 1:
        print('No (new) USB/COM ports found! Exiting!')
        exit()
    else:
        haptic_port = diff[0]
        print('Found', diff[0])

        old = new


def main():
    find_serial_ports()

    df = DetectionFields(lidar_port, gyro_port)
    hp = HapticHandler(haptic_port)

    df.start_updating()
    while df.thread_is_active:
        output = df.latest_output
        print(output)
        for o in output:
            if o is not None:
                hp.activate(o[2], o[0])
        time.sleep(1)


def serial_ports():
    """ Lists serial port names
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


if __name__ == '__main__':
    main()
    print('End of program')
