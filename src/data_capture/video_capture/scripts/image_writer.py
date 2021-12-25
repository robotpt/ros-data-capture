#!/usr/bin/env python
import argparse
import cv2
import datetime
import os
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class ImageWriter:

    VIDEO_TYPE_CODE = {
        'avi': cv2.VideoWriter_fourcc(*'mpeg'),
        'mp4': cv2.VideoWriter_fourcc(*'mpeg')
    }

    STD_DIMENSIONS = {
        "480p": (640, 480),
        "720p": (1280, 720),
        "1080p": (1920, 1080),
        "4k": (3840, 2160),
    }

    def __init__(
            self,
            image_topic,
            video_type,
            out_directory="videos"
    ):

        rospy.init_node('image_capture', anonymous=True)
        self._image_subscriber = rospy.Subscriber(image_topic, Image, self.write_image)

        self._frames_per_second = float(frames_per_second)
        self._video_type = video_type
        self._video_dimensions = self.STD_DIMENSIONS[video_dimensions]
        self._out_directory = out_directory
        self._video_writer = None

        self._is_record_subscriber = rospy.Subscriber(is_record_topic, Bool, self._is_record_callback)
        self._memory_watch_subscriber = rospy.Subscriber(
            is_memory_usage_exceeded_topic,
            Bool,
            self._memory_check_callback
        )
        self._is_recording = False
        self._allow_recording = True

        self._bridge = CvBridge()

    def _is_record_callback(self, data):
        is_record = data.data
        try:
            if self._allow_recording:
                if is_record:
                    self.start_writing()
                else:
                    self.stop_writing()

        except RuntimeError as e:
            rospy.logerr(e)

    def start_writing(self, out_file_name=None):
        if self._is_recording:
            raise RuntimeError("Video is already being written")

        if not os.path.exists(self._out_directory):
            os.makedirs(self._out_directory)
        if out_file_name is None:
            out_file_name = "{date_str}.{ext}".format(
                date_str=datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
                ext=self._video_type,
            )
        out_file_path = os.path.join(self._out_directory, out_file_name)

        self._video_writer = cv2.VideoWriter(
            out_file_path,
            self.VIDEO_TYPE_CODE[self._video_type],
            self._frames_per_second,  # Must be a float or output file is corrupted
            self._video_dimensions,
            True
        )

        self._is_recording = True

    def write_image(self, data):
        if data is not None and self._is_recording:
            try:
                cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                raise e

            self._video_writer.write(cv_image)

    def stop_writing(self):
        if not self._is_recording:
            raise RuntimeError("Video writing was not started")
        self._video_writer.release()
        self._is_recording = False

    def _memory_check_callback(self, data):
        is_memory_usage_exceeded = data.data

        if is_memory_usage_exceeded:
            self._allow_recording = False
            if self._video_writer is not None and self._video_writer.isOpened():
                self._video_writer.release()
                rospy.logerr(
                    "Stopped writing video due to memory utilization exceeded")
            else:
                rospy.loginfo(
                    "Memory utilization exceeded the set limits. Video writing will not happen")

        else:
            self._allow_recording = True


if __name__ == "__main__":

    # Getting the instance_id for the parameters
    parser = argparse.ArgumentParser(description='instance_id for video writing')
    parser.add_argument('--instance_id', help='instance_id for parameters namespace', default="1")
    args, _ = parser.parse_known_args()

    # Getting the values as params
    image_topic = rospy.get_param("/data_capture/"+args.instance_id+"/webcam_capture/default_param/image_topic", "webcam_capture/color/image_raw")
    is_record_topic = rospy.get_param("/data_capture/"+args.instance_id+"/webcam_capture/default_param/is_record_topic", "data_capture/is_record_interaction")
    output_directory = rospy.get_param("/data_capture/"+args.instance_id+"/webcam_capture/default_param/output_directory", "/root/upload")

    device_index = rospy.get_param("/data_capture/"+args.instance_id+"/webcam_capture/cam_settings/device_index", 0)
    frames_per_second = rospy.get_param("/data_capture/"+args.instance_id+"/webcam_capture/cam_settings/frames_per_second")
    video_type = rospy.get_param("/data_capture/"+args.instance_id+"/webcam_capture/cam_settings/video_type")
    video_dimensions = rospy.get_param("/data_capture/"+args.instance_id+"/webcam_capture/cam_settings/video_dimensions")

    is_memory_usage_exceeded_topic = rospy.get_param("/data_capture/"+args.instance_id+"/is_high_memory_usage_topic")

    ImageWriter(
        image_topic=image_topic,
        video_type=video_type,
        out_directory=output_directory
    )

    rospy.spin()
