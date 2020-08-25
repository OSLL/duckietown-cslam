#!/usr/bin/env python

import os
from collections import defaultdict
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np
from numpy.linalg import norm

import aruco_lib_adapter
import apriltags3
from image_rectifier import ImageRectifier

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class ApriltagAccuracyTester:

    def __init__(self, config):
        self.markers_trs_au = defaultdict(list)
        self.markers_trs_au_unique = defaultdict(list)
        self.markers_trs_at3 = defaultdict(list)
        self.markers_trs_at3_unique = defaultdict(list)

        self.arucoDetector = aruco_lib_adapter.Detector(
            detector_type=config['ACQ_DETECTOR_TYPE'],
            marker_size=config['ACQ_TAG_SIZE'],
            config_file=config['ACQ_CONFIG_YML'])

        self.apriltags3Detector = apriltags3.Detector(
            searchpath=[config['ACQ_APRILTAG_SO']],
            families='tag36h11',
            nthreads=4, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

    def process(self, config):
        bridge = CvBridge()
        bag = rosbag.Bag(config['INPUT_BAG_PATH'])
        image_topic_str = '/' + config['ACQ_DEVICE_NAME'] + '/' + config['ACQ_TOPIC_RAW']
        info_topic_str = '/' + config['ACQ_DEVICE_NAME'] + '/' + config['ACQ_TOPIC_CAMERAINFO']
        tag_size = config['ACQ_TAG_SIZE']

        camera_info = None
        for topic, msg, t in bag.read_messages(topics=[image_topic_str, info_topic_str]):
            if topic == info_topic_str:
                camera_info = msg
                break
        if camera_info is None:
            print("No info mgs")
            return

        img_rect_init = np.zeros((camera_info.width, camera_info.height))
        img_rect_init.shape = (camera_info.width, camera_info.height)
        K = np.array(camera_info.K).reshape((3, 3))
        D = camera_info.D
        image_rectifier = ImageRectifier(img_rect_init, K, D)

        for topic, img_msg, t in bag.read_messages(topics=[image_topic_str, info_topic_str]):
            print("%.2f / %.2f" % (t.to_sec() - bag.get_start_time(), bag.get_end_time() - bag.get_start_time()))
            if topic == image_topic_str:
                cv_image = bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding='mono8')

                if camera_info.height != cv_image.shape[0] or camera_info.width != cv_image.shape[1]:
                    print("Image dimensions differ from camera_info")
                    return

                rect_image, new_camera_matrix = image_rectifier.rectify(cv_image)
                camera_params = (new_camera_matrix[0, 0], new_camera_matrix[1, 1],
                                 new_camera_matrix[0, 2], new_camera_matrix[1, 2])

                if len(rect_image.shape) == 3:
                    rect_image_gray = cv2.cvtColor(rect_image, cv2.COLOR_BGR2GRAY)
                else:
                    rect_image_gray = rect_image

                tags_at3_list = self.apriltags3Detector.detect(rect_image_gray, True, camera_params, tag_size)
                tags_au_list = self.arucoDetector.detect(img_msg, K, D, camera_info.height, camera_info.width)

                tags_at3_dict = {m.tag_id: m for m in tags_at3_list}
                tags_au_dict = {m.tag_id: m for m in tags_au_list}
                tags = set([m.tag_id for m in tags_at3_list + tags_au_list])
                for tag_id in tags:
                    if tag_id in tags_at3_dict and tag_id in tags_au_dict:
                        self.markers_trs_at3[tag_id].append(tags_at3_dict[tag_id].pose_t)
                        self.markers_trs_au[tag_id].append(tags_au_dict[tag_id].pose_t)
                    elif tag_id in tags_at3_dict:
                        self.markers_trs_at3_unique[tag_id].append(tags_at3_dict[tag_id].pose_t)
                    elif tag_id in tags_au_dict:
                        self.markers_trs_au_unique[tag_id].append(tags_au_dict[tag_id].pose_t)

        bag.close()


def get_environment_variables():
    config = dict()

    config['ACQ_DEVICE_NAME'] = os.getenv('ACQ_DEVICE_NAME', 'watchtower01')
    config['ACQ_TOPIC_RAW'] = os.getenv('ACQ_TOPIC_RAW', 'camera_node/image/compressed')
    config['ACQ_TOPIC_CAMERAINFO'] = os.getenv('ACQ_TOPIC_CAMERAINFO', 'camera_node/camera_info')
    config['INPUT_BAG_PATH'] = os.getenv('INPUT_BAG_PATH')
    config['ACQ_DETECTOR_TYPE'] = os.getenv('ACQ_DETECTOR_TYPE', 'DFC')
    config['ACQ_TAG_SIZE'] = float(os.getenv('ACQ_TAG_SIZE', 0.065))
    config['ACQ_CONFIG_YML'] = os.getenv('ACQ_CONFIG_YML', 'config.yml')
    config['ACQ_APRILTAG_SO'] = os.getenv('ACQ_APRILTAG_SO', '/')

    return config


def draw(tr0, tr0_, tr1, tr1_):
    fig = plt.figure(figsize=(8, 8))
    plot1 = Axes3D(fig, proj_type='ortho')
    fig2 = plt.figure(figsize=(8, 8))
    plot2 = fig2.add_subplot(1, 1, 1)
    fig3 = plt.figure(figsize=(10, 4))
    plot3 = fig3.add_subplot(1, 1, 1)

    trs = [tr0, tr0_, tr1, tr1_]
    for i in range(4):
        for marker in trs[i]:
            trs[i][marker] = map(lambda t: [t[0], t[1], t[2] / 1.25], trs[i][marker])  # odd bug with z-scale

    trs_vals_list = list(tr0.values()) + list(tr0_.values()) + list(tr1.values()) + list(tr1_.values())
    trs_vals = [item for sublist in trs_vals_list for item in sublist]
    max_x = max(t[0] for t in trs_vals)
    min_x = min(t[0] for t in trs_vals)
    max_y = max(t[1] for t in trs_vals)
    min_y = min(t[1] for t in trs_vals)
    max_z = max(t[2] for t in trs_vals)
    min_z = min(t[2] for t in trs_vals)
    mid_x = (max_x + min_x) / 2
    mid_y = (max_y + min_y) / 2
    mid_z = (max_z + min_z) / 2
    side2 = max(max_x - min_x, max_y - min_y, max_z - min_z) / 2 * 1.2
    plot1.set_xlim3d(mid_x - side2, mid_x + side2)
    plot1.set_ylim3d(mid_y - side2, mid_y + side2)
    plot1.set_zlim3d(mid_z - side2, mid_z + side2)
    plot2.set_xlim(mid_x - side2, mid_x + side2)
    plot2.set_ylim(mid_y - side2, mid_y + side2)

    colors = ['blue', 'cyan', 'red', 'magenta']
    for i in range(4):
        for marker in trs[i]:
            marker_trs = trs[i][marker]
            tr_x = [t[0] for t in marker_trs]
            tr_y = [t[1] for t in marker_trs]
            tr_z = [t[2] for t in marker_trs]
            plot1.scatter(tr_x, tr_y, tr_z, marker='o', color=colors[i])
            plot2.plot(tr_x, tr_y, marker='o', color=colors[i])

    plot1.set_xlabel('x (m)')
    plot1.set_ylabel('y (m)')
    plot1.set_zlabel('z (m)')
    plot2.set_xlabel('x (m)')
    plot2.set_ylabel('y (m)')

    diff = ([], [])
    for marker in trs[0]:
        marker_trs_0 = trs[0][marker]
        marker_trs_1 = trs[2][marker]
        for i in range(len(marker_trs_0)):
            diff[0].append(norm([marker_trs_0[i][0], marker_trs_0[i][1]]))
            diff[1].append(norm([marker_trs_1[i][j] - marker_trs_0[i][j] for j in range(3)]))

    plot3.plot(diff[0], diff[1], 'go', markersize=0.5)
    plot3.set_xlabel('distance to z-axis (m)')
    plot3.set_ylabel('distance between positions (m)')

    plt.pause(10000)


def main():
    config = get_environment_variables()
    aat = ApriltagAccuracyTester(config)
    aat.process(config)

    draw(aat.markers_trs_at3, aat.markers_trs_at3_unique, aat.markers_trs_au, aat.markers_trs_au_unique)


if __name__ == '__main__':
    main()
