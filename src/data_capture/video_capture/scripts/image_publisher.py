#!/usr/bin/env python

import argparse
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from image_capture import ImageCapture


class ImagePublisher:

    def __init__(
            self,
            image_topic,
            device_index,
            is_record_topic
    ):

        rospy.init_node('image_capture', anonymous=True)
        self._image_capture = ImageCapture(
            device_index=device_index
        )

        self._is_record_subscriber = rospy.Subscriber(
            is_record_topic, Bool, self._is_record_callback)
        self._image_publisher = rospy.Publisher(image_topic, Image, queue_size=1)

        self._is_recording = False

    def _is_record_callback(self, data):
        is_record = data.data
        try:
            if is_record:
                self._is_recording = True
                rospy.loginfo("Starting to record video")
                self._image_capture.start_recording()

            else:
                self._is_recording = False
                if self._image_capture.is_recording:
                    rospy.loginfo("Stopped recording video")
                    self._image_capture.stop_recording()

        except RuntimeError as e:
            rospy.logerr(e)

    def publish_image(self):
        if self._is_recording:
            image = self._image_capture.get_image(is_throw_error_if_not_recording=False)
            try:
                image = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
                self._image_publisher.publish(image)
            except Exception as e:
                rospy.loginfo("Error: {error}".format(error=e))


if __name__ == "__main__":
    # Getting the instance_id for the parameters
    parser = argparse.ArgumentParser(description='instance_id for video publishing')
    parser.add_argument('--instance_id', help='instance_id for parameters namespace', default="1")
    args, _ = parser.parse_known_args()

    # Getting the values as params
    image_topic = rospy.get_param(
        "/data_capture/"+args.instance_id+"/webcam_capture/default_param/image_topic", "webcam_capture/color/image_raw")
    is_record_topic = rospy.get_param(
        "/data_capture/" + args.instance_id + "/webcam_capture/default_param/is_record_topic",
        "video_capture/is_record")

    device_index = rospy.get_param("/data_capture/webcam_capture/cam_settings/device_index", 3)
    frames_per_second = rospy.get_param(
        "/data_capture/" + args.instance_id + "/webcam_capture/cam_settings/frames_per_second")

    is_memory_usage_exceeded_topic = rospy.get_param(
        "/data_capture/" + args.instance_id + "/is_high_memory_usage_topic")

    image_publisher = ImagePublisher(
        image_topic=image_topic,
        device_index=device_index,
        is_record_topic=is_record_topic
    )

    while not rospy.is_shutdown():
        image_publisher.publish_image()
        rospy.sleep(1/frames_per_second)
