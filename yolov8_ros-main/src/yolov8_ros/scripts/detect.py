#!/usr/bin/env python3

import cv2
import rospy
from cob_perception_msgs.msg import Detection, DetectionArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO


class Detector:
    def __init__(self):
        self.model = YOLO("yolov8n.pt")

        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.callback, queue_size=1
        )

        self.cob_detection_pub = rospy.Publisher(
            "/yolov8/cob_detections", DetectionArray, queue_size=10
        )

        self.bridge = CvBridge()

    def callback(self, data):
        im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # im = cv2.flip(im, 1)
        res = self.model(im, verbose=False)[0]
        im0 = res.plot()

        detection_array = DetectionArray()
        detection_array.header = data.header

        if len(res.boxes):
            for box in res.boxes:
                detection = Detection()

                # cob_perception_msgs/Detection Message
                detection.header = data.header
                detection.label = str(box.cls.item())
                # detection.id = int(box.id)
                detection.detector = "yolov8"
                detection.score = box.conf.item()

                # xywh
                detection.mask.roi.x = int(box.xyxy[0][0].item())
                detection.mask.roi.y = int(box.xyxy[0][1].item())
                detection.mask.roi.width = int(box.xywh[0][2].item())
                detection.mask.roi.height = int(box.xywh[0][3].item())

                # # xy
                detection.bounding_box_lwh.x = box.xywh[0][0].item()
                detection.bounding_box_lwh.y = box.xywh[0][1].item()

                detection_array.detections.append(detection)

        self.cob_detection_pub.publish(detection_array)

        cv2.imshow("sample", im0)
        cv2.waitKey(1)


if __name__ == "__main__":

    rospy.init_node("yolov8", anonymous=True)
    detector = Detector()

    rospy.spin()
