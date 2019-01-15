#!/usr/bin/python
# coding=utf-8

"""camera snap"""

import cv2
import time
import rospy
import json
import logging
import numpy as np
from os.path import join
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from dji.telemetry import telemetry

logging.basicConfig(format='%(asctime)s %(levelname)s [%(module)s] %(message)s', level=logging.INFO)
log = logging.getLogger()


def snap(width=3840, height=2160, channel=3, flag=True, save_dir="static/data/"):
    rospy.init_node("snap_image", anonymous=True)
    c = 0
    while flag:
        suffix = datetime.now().strftime('%Y%m%d_%H%M%S')
        for i in range(2,3):
            topic_name = "/csi_cam_%d" % i
            #raw_img = rospy.client.wait_for_message(topic_name + "/image_raw", Image, 10)
            params = rospy.get_param(topic_name)
            img = np.frombuffer(raw_img.data, dtype=np.uint8)
            img = img.reshape([height, width, channel])
            img_name = "IMG_" + suffix + "_" + str(c).zfill(4) + "_" + camera_band(params["camera_name"]) + ".jpg"
            cv2.imwrite(join(save_dir, "images", img_name), img)
        pose_dict = telemetry()
        print pose_dict
        with open(join(save_dir, "json", "{}_{}.json".format(suffix, str(c).zfill(4))), "w") as fw:
            json.dump(pose_dict, fw)
        c += 1
        time.sleep(1)


def image_callback(msg, width=3840, height=2160, channel=3, save_dir="static/data/"):
    suffix = datetime.now().strftime('%Y%m%d_%H%M%S')
    #img = np.frombuffer(msg.data, dtype=np.uint8)  # solution 1
    img = bridge.imgmsg_to_cv2(msg, "rgb8")  # solution 2
    #img = cv_bridge.toCvShare(msg, "rgb8")  # solution 2
    #img = img.reshape([height, width, channel])
    img_name = "IMG_" + suffix + "_" + ".jpg"
    cv2.imwrite(join(save_dir, "images", img_name), img)
    pose_dict = telemetry()
    print pose_dict
    with open(join(save_dir, "json", "{}.json".format(suffix)), "w") as fw:
        json.dump(pose_dict, fw)


def camera_band(camera_name):
    """相机波段对应的名称"""
    band_dict = dict()
    band_dict["csi_cam_0"] = "NIR"  # 近红外 wave length 840
    band_dict["csi_cam_1"] = "GRE"  # 绿光 560
    band_dict["csi_cam_2"] = "RGB"  # RGB正常照片
    band_dict["csi_cam_3"] = "RED"  # 红光 668
    # band_dict["csi_cam_4"] = "REDEdge"  # 红边光 717
    return band_dict[camera_name]


def main():
    # save image solution 2
    topic_name = "/csi_cam_2" 
    rospy.init_node("snap_image", anonymous=True)
    global bridge
    bridge = CvBridge()
    sub_img = rospy.Subscriber(topic_name + "/image_raw", Image, image_callback)
    rospy.spin()

    # save image solution 1
    #snap()
    return


if __name__ == "__main__":
    main()
