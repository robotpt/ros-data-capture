#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

import argparse

from video_recorder import VideoRecorder

from threading import Timer
import os
from time import sleep


"""
This is a threaded timer to execute a piece of code periodically 
without blocking the main execution.

This non-blocking method allows the code to keep running while also
executing our function every n seconds

NOTE:
- start() and stop() are safe to call multiple times even if the timer 
has already started/stopped

- You can change interval anytime, it will be effective after next run. 
Same for args, kwargs and even function!
"""

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

"""
This function is used to monitor the filesize of a file and throw an error 
if it exceeds a preset value.

Arguments:
filename: The filename of the video file with full path to it. (str)
upperlimit_bytes: The upperlimit of the filesize (int)
"""
def monitorVideoFileSize(filename, upperlimit_bytes):
    filesize_in_bytes = 0
    try:
        filesize_in_bytes = os.path.getsize(filename)
    except FileNotFoundError:
        # During the first run, the function is called before the file is created. So skip the check
        pass
    
    if filesize_in_bytes > upperlimit_bytes:
        raise RuntimeError("The video file size has exceeded the set limits")


class VideoCapture:

    def __init__(
            self,
            is_record_topic,
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

        self._is_record_subscriber = rospy.Subscriber(is_record_topic, Bool, self._is_record_callback)
        self._image_subscriber = rospy.Subscriber(image_topic, Image, self._image_callback)
        self._bridge = CvBridge()

    def _image_callback(self, data):

        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e

        self._video_recorder.add_image(cv_image, is_throw_error_if_not_recording=False)

    def _is_record_callback(self, data):

        is_record = data.data
        try:
            if is_record:
                rospy.loginfo("Starting to record video")             

                # Starting the periodic check on filesize. Upperfilesize = 1GB, interval=5 seconds
                # RepeatedTimer auto-starts, no need of rt.start()
                rt = RepeatedTimer(5, monitorVideoFileSize, self._video_recorder.out_file_path, 1073741824) 

                self._video_recorder.start_recording()

            else:
                rospy.loginfo("Stopped recording video")
                self._video_recorder.stop_recording()
                rt.stop()
        except RuntimeError as e:
            # Checking if the rt variable has been defined before stopping it
            if 'rt' in locals() or 'rt' in globals():
                rt.stop()
            rospy.logerr(e)


if __name__ == "__main__":

    # Having the topic as a command line argument allows multiple video recorder nodes to exist on different topics
    parser = argparse.ArgumentParser(description='Record video from an image topic')
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
        video_type=args.video_type,
        video_dimensions=args.video_dimensions,
        frames_per_second=args.frames_per_second,
        out_directory=args.output_directory,
    )

    rospy.spin()
