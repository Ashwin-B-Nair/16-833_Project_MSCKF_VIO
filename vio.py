from queue import Queue
from threading import Thread
import threading

from config import ConfigEuRoC
from image import ImageProcessor
from msckf import MSCKF

import numpy as np

# Create an event to pause and resume threads
pause_event = threading.Event()

class VIO(object):
    def __init__(self, config, img_queue, imu_queue, gt_queue, viewer=None):
        self.config = config
        self.viewer = viewer

        self.img_queue = img_queue
        self.imu_queue = imu_queue
        self.gt_queue = gt_queue
        self.feature_queue = Queue()

        self.image_processor = ImageProcessor(config)
        self.msckf = MSCKF(config)

        self.img_thread = Thread(target=self.process_img)
        self.imu_thread = Thread(target=self.process_imu)
        self.vio_thread = Thread(target=self.process_feature)
        self.img_thread.start()
        self.imu_thread.start()
        self.vio_thread.start()

    def process_img(self):
        while True:
            img_msg = self.img_queue.get()
            if img_msg is None:
                self.feature_queue.put(None)
                return
            # print('img_msg', img_msg.timestamp)

            if self.viewer is not None:
                self.viewer.update_image(img_msg.cam0_image)

            feature_msg = self.image_processor.stareo_callback(img_msg)

            if feature_msg is not None:
                self.feature_queue.put(feature_msg)

    def process_imu(self):
        while True:
            imu_msg = self.imu_queue.get()
            if imu_msg is None:
                return
            # print('imu_msg', imu_msg.timestamp)

            self.image_processor.imu_callback(imu_msg)
            self.msckf.imu_callback(imu_msg)

    def process_feature(self):
        gt_time_list = []    # Stores all gt timestamp values from gt queue
        gt_position_list = []
        while True:
            feature_msg = self.feature_queue.get()

            # print("gt val type", type(gt_val))

            if feature_msg is None:
                return
            
            # print('feature_msg', feature_msg.timestamp)
            result = self.msckf.feature_callback(feature_msg)

            while not self.gt_queue.empty():
                gt_msg = self.gt_queue.get()        # When this is done, elements are removed from the gt queue
                if not gt_msg == None:
                    gt_time_list.append(gt_msg.timestamp)
                    gt_position_list.append(gt_msg.p)

            index_start = np.where(np.array(gt_time_list) >= result.timestamp)[0][0]    # GT queue is larger because images data keeps getting published to it even while the data in the imu and image functions are being processed
            # print(index_start)
            gt_time = gt_time_list[index_start] # dataset.groundtruth.parse(gt_msg)
            gt_pos = gt_position_list[index_start]

            if result is not None and self.viewer is not None:
                self.viewer.update_pose(result.cam0_pose,result.timestamp)
                # self.viewer.update_timestamp(result.timestamp)      # Timestamp is in nanoseconds
                # print("published timestamp", result.timestamp)
                self.viewer.update_gt_pose(gt_pos,gt_time)
    
if __name__ == '__main__':
    import time
    import argparse

    from dataset import EuRoCDataset, DataPublisher
    from viewer import Viewer

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, default=r"C:\Users\athar\OneDrive\Documents\Sem-1\SLAM\Project\codes\MH_01_easy", 
        help='Path of EuRoC MAV dataset.')
    parser.add_argument('--view', action='store_true', help='Show trajectory.')
    args = parser.parse_args()

    if args.view:
        viewer = Viewer()
        print('viewer on')
    else:
        viewer = None
        print('viewer not enabled')

    dataset = EuRoCDataset(args.path)
    dataset.set_starttime(offset=40.)   # start from static state

    img_queue = Queue()
    imu_queue = Queue()     # queue.Queue is part of Python's standard library (queue module) and is used for thread-safe data sharing.
    gt_queue = Queue()    # Use Case: It supports operations like put() and get() to safely enqueue and dequeue data between threads.

    config = ConfigEuRoC()
    msckf_vio = VIO(config, img_queue, imu_queue, gt_queue, viewer=viewer)

    duration = float('inf')
    ratio = 1  # make it smaller if image processing and MSCKF computation is slow
    imu_publisher = DataPublisher(
        dataset.imu, imu_queue, duration, ratio)
    img_publisher = DataPublisher(
        dataset.stereo, img_queue, duration, ratio)
    gt_publisher = DataPublisher(
        dataset.groundtruth, gt_queue, duration,ratio)

    now = time.time()
    imu_publisher.start(now)
    img_publisher.start(now)
    gt_publisher.start(now)