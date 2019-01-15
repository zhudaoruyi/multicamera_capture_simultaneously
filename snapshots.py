#!/usr/bin/python
# coding=utf-8

"""多摄像头定时拍摄"""

import os
import cv2
import time
import logging
from datetime import datetime

logging.basicConfig(format='%(asctime)s %(levelname)s [%(module)s] %(message)s', level=logging.INFO)
log = logging.getLogger()


def open_onboard_camera(device_number, width, height):
    return cv2.VideoCapture("nvcamerasrc sensor-id=%s ! video/x-raw(memory:NVMM), width=(int)%s, height=(int)%s, "
                            "format=(string)I420, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, "
                            "format=(string)BGRx ! videoconvert ! video/x-raw, "
                            "format=(string)BGR ! appsink" % (device_number, width, height))


def num_series(path):
    files_dir = os.listdir(path)
    files_num = len(files_dir)
    if files_num == 0:
        return str(0).zfill(4)
    else:
        series_n = [int(name.split('.')[0].split('_')[-1]) for name in files_dir]
        max_num = max(series_n)
        return str(max_num).zfill(4)


def cam_band(camera_id):
    """相机波段对应的名称"""
    band_dict = dict()
    band_dict["0"] = "NIR"  # 近红外 wave length 840
    band_dict["1"] = "GRE"  # 绿光 560
    band_dict["2"] = "RGB"  # RGB正常照片
    band_dict["3"] = "RED"  # 红光 668
    # band_dict["csi_cam_4"] = "REDEdge"  # 红边光 717
    return band_dict[camera_id]


def snap_cam(intervals=1, width=3840, height=2160, flag=True, save_dir="static/data/"):
    camera_band = {"0": "NIR", "1": "GRE", "2": "RGB", "3": "RED"}
    save_dir = os.path.join(save_dir, "images")
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    video_0 = open_onboard_camera(0, width, height)
    video_1 = open_onboard_camera(1, width, height)
    video_2 = open_onboard_camera(2, width, height)
    video_3 = open_onboard_camera(3, width, height)
    log.info("video opened.")
    try:
        count = 0
        while flag:
            _, frame_0 = video_0.read()
            _, frame_1 = video_1.read()
            _, frame_2 = video_2.read()
            _, frame_3 = video_3.read()
            log.info("video read.")

            suffix = datetime.now().strftime('%Y%m%d_%H%M%S')
            img_name_0 = "IMG_" + suffix + "_" + str(count).zfill(4) + "_" + camera_band["0"] + ".jpg"
            img_name_1 = "IMG_" + suffix + "_" + str(count).zfill(4) + "_" + camera_band["1"] + ".jpg"
            img_name_2 = "IMG_" + suffix + "_" + str(count).zfill(4) + "_" + camera_band["2"] + ".jpg"
            img_name_3 = "IMG_" + suffix + "_" + str(count).zfill(4) + "_" + camera_band["3"] + ".jpg"
            log.info("image named.")

            cv2.imwrite(os.path.join(save_dir, img_name_0), frame_0)
            cv2.imwrite(os.path.join(save_dir, img_name_1), frame_1)
            cv2.imwrite(os.path.join(save_dir, img_name_2), frame_2)
            cv2.imwrite(os.path.join(save_dir, img_name_3), frame_3)
            log.info("video captured {}".format(count))

            count += 1
            time.sleep(intervals)  # 拍照间隔时间
    except KeyboardInterrupt:
        video_0.release()
        video_1.release()
        video_2.release()
        video_3.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    """
    intervals: 拍照时间间隔，可设置为1秒，12分钟500次（一次4张）照片
    flag     : 开始拍照的条件，可根据航点任务是否在执行来决定是否拍照，flag=flight_status.data
    """
    # snap_cam(width=3840, height=2160)  # 会死掉
    snap_cam(intervals=1, width=1920, height=1080, flag=True)  # 很坚挺
