#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

import argparse

from video_recorder import VideoRecorder


class VideoCapture:

    def __init__(
            self,
            is_record_topic,
            is_memory_usage_exceeded_topic,
            image_topic,
            video_type,
            video_dimensions,
            frames_per_second,
            out_directory,
    ):

        rospy.init_node('video_recorder', anonymous=True)
        self._video_recorder = VideoRecorder(
            video_type=video_type,
            video_dimensions=video_dimensions,
            frames_per_second=frames_per_second,
            out_directory=out_directory,
        )

        self._is_record_subscriber = rospy.Subscriber(
            is_record_topic, Bool, self._is_record_callback)
        self._image_subscriber = rospy.Subscriber(
            image_topic, Image, self._image_callback)
        self._memory_watch_subscriber = rospy.Subscriber(
            is_memory_usage_exceeded_topic, Bool, self._memory_check_callback)
        self._bridge = CvBridge()

        # This flag is used to block recording if memory exceeds limits
        self._allow_recording = True

    def _image_callback(self, data):

        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e

        self._video_recorder.add_image(
            cv_image, is_throw_error_if_not_recording=False)

    def _is_record_callback(self, data):

        is_record = data.data
        try:
            if is_record:
                if self._allow_recording:
                    rospy.loginfo("Starting to record video")
                    self._video_recorder.start_recording()
                else:
                    rospy.logerr(
                        "Recording will not happen due to memory limits exceeded")

            else:
                if self._video_recorder._is_recording:
                    rospy.loginfo("Stopped recording video")
                    self._video_recorder.stop_recording()

        except RuntimeError as e:
            rospy.logerr(e)

    def _memory_check_callback(self, data):
        is_memory_usage_exceeded = data.data

        if is_memory_usage_exceeded:
            self._allow_recording = False
            if self._video_recorder._is_recording:
                self._video_recorder.stop_recording()
                rospy.logerr(
                    "Stopped Video recording due to memory utilization exceeded")
            else:
                rospy.loginfo(
                    "Memory utilization exceeded the set limits. Recording will not happen")

        else:
            self._allow_recording = True


if __name__ == "__main__":

    # Having the topic as a command line argument allows multiple video recorder nodes to exist on different topics
    parser = argparse.ArgumentParser(
        description='Record video from an image topic')
    parser.add_argument('--image-topic', help='The image topic for the program to subscribe to',
                        default="camera/color/image_raw")
    parser.add_argument('--output-directory', help='Directory where videos should be saved to',
                        default="/root/videos")
    parser.add_argument('--frames-per-second', type=float, help='Number of frames per second sent from the image topic',
                        default=30.0)
    parser.add_argument('--is-record-topic', help='Topic that publishes if recordings should start or stop',
                        default="video_capture/is_record")
    parser.add_argument('--video-type', help='Format of the video to be saved',
                        default="mp4")
    parser.add_argument('--video-dimensions', help='Dimensions of the video to be saved',
                        default="480p")
    args, _ = parser.parse_known_args()

    VideoCapture(
        image_topic=args.image_topic,
        is_record_topic=args.is_record_topic,
        is_memory_usage_exceeded_topic=rospy.get_param(
            "data_capture/is_high_memory_usage_topic"),
        video_type=args.video_type,
        video_dimensions=args.video_dimensions,
        frames_per_second=args.frames_per_second,
        out_directory=args.output_directory,
    )

    rospy.spin()
