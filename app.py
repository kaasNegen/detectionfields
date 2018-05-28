from detectionfields import *
import time

if __name__ == '__main__':
    x = DetectionFields()
    x.start_updating()
    while True:
        print(x.latest_output)
        time.sleep(1)