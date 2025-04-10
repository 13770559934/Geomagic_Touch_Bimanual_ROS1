#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def find_available_cameras():
    available_cameras = []
    for camera_id in range(10): 
        print(camera_id)
        cap = cv2.VideoCapture(camera_id)
        if cap.isOpened():
            print(cap.isOpened())
            available_cameras.append(camera_id)
            cap.release()
        else:
            continue
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

        self.rate = rospy.Rate(rate) # rate Hz, rate can be a parameter from parameter??

    def publish_images(self):

        cam_objs = []

        for camera_id in self.camera_ids:
            # open camera
            temp = []
            cap = cv2.VideoCapture(camera_id)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) #WHY???

            temp.append(cap)
            temp.append(camera_id)
            cam_objs.append(temp)

        while not rospy.is_shutdown():
            rospy.loginfo(self.camera_ids)
            for camera_temp in cam_objs:
                # open camera
                cap = camera_temp[0]
                camera_id = camera_temp[1]

                if not cap.isOpened():
                    rospy.logerr(f"Failed to open camera {camera_id}")
                    continue
                
                # read a frame
                ret, frame = cap.read()

                if frame.dtype != np.uint8:
                    frame = frame.astype(np.uint8)

                print(type(frame), frame.shape, np.sum(frame), frame.dtype )

                 # 显示图像
                cv2.imshow(f'Camera {camera_id}', frame)

                # 检测按键
                if cv2.waitKey(1) & 0xFF == ord('q'):  # 按下 'q' 键退出
                    cv2.destroyAllWindows()
                    break
                
                if not ret:
                    rospy.logerr(f"Failed to read frame from camera {camera_id}")
                    continue

                #OpenCV image ---> ROS msg
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                except CvBridgeError as e:
                    rospy.logerr(f"CvBridge Error for camera {camera_id}: {e}")
                    continue

                # publish image
                self.publishers[camera_id].publish(ros_image)

                rospy.loginfo( cap.get(cv2.CAP_PROP_FPS))

            self.rate.sleep()


if __name__ == "__main__":
    try:
        camera_ids = [0] # we can change this to getting parameter from ros
        rospy.loginfo('camera_ids')
        camera_publisher = CameraPublisher(camera_ids)
        camera_publisher.publish_images()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass