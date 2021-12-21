import cv2
import datetime
import os


class ImageCapture:

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
            device_index=3,
            video_type='mp4',
            video_dimensions='480p',
            frames_per_second=30.0,
    ):

        self._device_index = device_index
        self._video_dimensions = self.STD_DIMENSIONS[video_dimensions]
        self._video_capture = cv2.VideoCapture(self._device_index + cv2.CAP_FFMPEG)
        self._video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._video_dimensions[0])
        self._video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._video_dimensions[1])
        self._video_capture.set(cv2.CAP_PROP_FPS, frames_per_second)
        self._video_capture.set(cv2.CAP_PROP_FOURCC, self.VIDEO_TYPE_CODE[video_type])
        self._video_capture.set(cv2.CAP_PROP_CONVERT_RGB, True)
        self._video_capture.set(cv2.CAP_PROP_MODE, cv2.CAP_MODE_YUYV)

        self._is_recording = False

    def start_recording(self):
        if self._is_recording:
            raise RuntimeError("Video is already being recorded")

        self._is_recording = True
        self._video_capture.open(self._device_index)
        if not self._video_capture.isOpened():
            print("Could not open video device at index " + str(self._device_index))

        if self._video_capture is None or not self._video_capture.isOpened():
            print("Warning: unable to open video source")

    def get_image(self, is_throw_error_if_not_recording=True):
        if self._is_recording:
            ret, image = self._video_capture.read()
            return image
        else:
            if is_throw_error_if_not_recording:
                raise RuntimeError("Recording has not been started")

    def stop_recording(self):
        if not self._is_recording:
            raise RuntimeError("Video recording was not started")

        self._video_capture.release()
        self._is_recording = False

    @property
    def is_recording(self):
        return self._is_recording
