#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def find_available_cameras():
    available_cameras = []
    for camera_id in range(10):  # 尝试检查前 10 个可能的相机 ID
        print(camera_id)
        cap = cv2.VideoCapture(camera_id)
        if cap.isOpened():
            print(cap.isOpened())
            available_cameras.append(camera_id)
            cap.release()
        else:
            continue  # 如果无法打开当前 ID 的相机，后续的 ID 也不太可能有效
    return available_cameras



class CameraPublisher:
    def __init__(self, camera_ids, rate = 60):
        self.camera_ids = camera_ids  # camera ID list
        self.bridge = CvBridge()
        self.publishers = {}  # dict for every publisher object

        rospy.init_node("camera_publisher", anonymous=True)

        # one topic per cameras
        for camera_id in self.camera_ids:
            topic_name = f"/camera{camera_id}/image_raw"
            self.publishers[camera_id] = rospy.Publisher(topic_name, Image, queue_size=10)

        self.rate = rate # rate Hz, rate can be a parameter from parameter??

    def publish_images(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.camera_ids)
            for camera_id in self.camera_ids:
                # open camera
                cap = cv2.VideoCapture(camera_id)
                if not cap.isOpened():
                    rospy.logerr(f"Failed to open camera {camera_id}")
                    continue
                
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) #WHY???
                # read a frame
                ret, frame = cap.read()
                if not ret:
                    rospy.logerr(f"Failed to read frame from camera {camera_id}")
                    cap.release()
                    continue

                #OpenCV image ---> ROS msg
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                except CvBridgeError as e:
                    rospy.logerr(f"CvBridge Error for camera {camera_id}: {e}")
                    continue

                # publish image
                self.publishers[camera_id].publish(ros_image)
                cap.release()

                rospy.loginfo( cap.get(cv2.CAP_PROP_FPS))

            # self.rate.sleep()

if __name__ == "__main__":
    try:
        # 从参数服务器或命令行获取相机 ID 列表
        camera_ids = find_available_cameras() # we can change this to getting parameter from ros
        rospy.loginfo(camera_ids)
        camera_publisher = CameraPublisher(camera_ids)
        camera_publisher.publish_images()
    except rospy.ROSInterruptException:
        pass