import logging
import os
from collections import defaultdict
import rospy
import rosbag
import time
import cv2
from cv_bridge import CvBridge
import numpy as np
from numpy.linalg import norm
from sensor_msgs.msg import (CameraInfo, CompressedImage)

from duckietown_msgs.msg import AprilTagExtended

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class ApriltagAccuracyTester:

    def __init__(self, config):
        print("init_node", flush=True)

        rospy.init_node('tester_node')
        print("1", flush=True)

        self.img_comp_topic1 = '/watchtower10/camera_node/image/compressed'
        self.cam_info_topic1 = '/watchtower10/camera_node/camera_info'
        self.img_comp_topic2 = '/watchtower20/camera_node/image/compressed'
        self.cam_info_topic2 = '/watchtower20/camera_node/camera_info'

        self.apriltags_topic = '/poses_acquisition/poses'
        print("2", flush=True)

        self.comp_img_pub1 = rospy.Publisher(self.img_comp_topic1, CompressedImage, queue_size=1000)
        self.cam_info_pub1 = rospy.Publisher(self.cam_info_topic1, CameraInfo,      queue_size=1)
        self.comp_img_pub2 = rospy.Publisher(self.img_comp_topic2, CompressedImage, queue_size=1000)
        self.cam_info_pub2 = rospy.Publisher(self.cam_info_topic2, CameraInfo,      queue_size=1)
        print("3", flush=True)

        self.apriltags_sub = rospy.Subscriber(self.apriltags_topic, AprilTagExtended,
                                              self.apriltags_callback, queue_size=50)
        print("4", flush=True)
        self.markers_trs_au = defaultdict(list)
        self.markers_trs_au_unique = defaultdict(list)
        self.markers_trs_at = defaultdict(list)
        self.markers_trs_at_unique = defaultdict(list)
        print("5", flush=True)

        self.apriltag_msgs = []
        self.req_timeout = 0.5
        print("6", flush=True)

        print("init")
        time.sleep(5)
        print("start")

    def apriltags_callback(self, apriltag_msg):
        self.apriltag_msgs.append(apriltag_msg)

    def request(self, comp_img_pub, img_msg):
        self.apriltag_msgs = []
        comp_img_pub.publish(img_msg)
        time.sleep(self.req_timeout)
        return self.apriltag_msgs

    def process(self, config):
        print("process", flush=True)
        bag = rosbag.Bag(config['INPUT_BAG_PATH'])
        image_topic_str = '/' + config['ACQ_DEVICE_NAME'] + '/' + config['ACQ_TOPIC_RAW']
        info_topic_str = '/' + config['ACQ_DEVICE_NAME'] + '/' + config['ACQ_TOPIC_CAMERAINFO']

        camera_info = None
        print("read", flush=True)
        for topic, msg, t in bag.read_messages(topics=[image_topic_str, info_topic_str]):
            print("### %.2f / %.2f" % (t.to_sec() - bag.get_start_time(), bag.get_end_time() - bag.get_start_time()), flush=True)
            if topic == info_topic_str:
                camera_info = msg
                break
        if camera_info is None:
            print("No info mgs")
            return
        print("found", flush=True)

        self.cam_info_pub1.publish(camera_info)
        self.cam_info_pub2.publish(camera_info)
        time.sleep(1)

        for topic, img_msg, t in bag.read_messages(topics=[image_topic_str, info_topic_str]):
            # print("%.2f / %.2f" % (t.to_sec() - bag.get_start_time(), bag.get_end_time() - bag.get_start_time()), flush=True)
            if topic == image_topic_str:
                print("img pub", flush=True)

                tags_at_list = self.request(self.comp_img_pub1, img_msg)
                print(tags_at_list, flush=True)
                tags_au_list = [] # self.request(self.comp_img_pub2, img_msg)

                tags_at_dict = {m.tag_id: m for m in tags_at_list}
                tags_au_dict = {m.tag_id: m for m in tags_au_list}
                tags = set([m.tag_id for m in tags_at_list + tags_au_list])
                for tag_id in tags:
                    if tag_id in tags_at_dict and tag_id in tags_au_dict:
                        self.markers_trs_at[tag_id].append(tags_at_dict[tag_id].pose_t)
                        self.markers_trs_au[tag_id].append(tags_au_dict[tag_id].pose_t)
                    elif tag_id in tags_at_dict:
                        self.markers_trs_at_unique[tag_id].append(tags_at_dict[tag_id].pose_t)
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
    logging.warning("main")
    config = get_environment_variables()
    aat = ApriltagAccuracyTester(config)
    aat.process(config)

    draw(aat.markers_trs_at, aat.markers_trs_at_unique, aat.markers_trs_au, aat.markers_trs_au_unique)


if __name__ == '__main__':
    main()