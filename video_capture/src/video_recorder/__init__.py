import cv2
import datetime
import os


class VideoRecorder:

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
            video_type='mp4',
            video_dimensions='480p',
            frames_per_second=30.0,
            out_directory="videos"
    ):

        if video_type not in self.VIDEO_TYPE_CODE:
            raise KeyError(
                'Invalid video type, please use one of the following: {}'.format(self.VIDEO_TYPE_CODE.keys())
            )
        if video_dimensions not in self.STD_DIMENSIONS:
            raise KeyError(
                'Invalid video dimensions, please use one of the following: {}'.format(self.STD_DIMENSIONS.keys())
            )

        self._video_type = video_type
        self._video_dimensions = self.STD_DIMENSIONS[video_dimensions]
        self._frames_per_second = float(frames_per_second)

        self._out_directory = out_directory

        self._is_recording = False
        self._video_writer = None

    def start_recording(self, out_file_name=None):

        if self._is_recording:
            raise RuntimeError("Video is already being recorded")

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
            True,
        )
        self._is_recording = True

    def add_image(self, image, is_throw_error_if_not_recording=True):

        if self._is_recording:
            self._video_writer.write(image)
        else:
            if is_throw_error_if_not_recording:
                raise RuntimeError("Recording has not been started")

    def stop_recording(self):

        if not self._is_recording:
            raise RuntimeError("Video recording was not started")

        self._video_writer.release()
        self._video_writer = None
        self._is_recording = False
