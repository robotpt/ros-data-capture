#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from audio_common_msgs.msg import AudioData

import os

import pyaudio
import wave
import datetime


IS_RECORD_TOPIC = rospy.get_param('messages/topics/is_record')
AUDIO_DATA_TOPIC = rospy.get_param('messages/topics/audio_data')
OUT_DIRECTORY = rospy.get_param('output_directory')

CHANNEL = rospy.get_param('channels')
SAMPLE_RATE = rospy.get_param('sample_rate')
CHUNK_SIZE = rospy.get_param('chunk_size')
FORMAT_SIZE = pyaudio.paInt16


class AudioCapture:

    def __init__(
            self,
            is_record_topic,
            audio_data_topic,
            out_file_directory='audio',
    ):

        rospy.init_node('audio_recorder', anonymous=True)

        self._is_record_subscriber = rospy.Subscriber(is_record_topic, Bool, self._record_callback, queue_size=1)
        self._audio_data_topic = rospy.Subscriber(audio_data_topic, AudioData, self._audio_data_callback, queue_size=1)

        self._out_directory = out_file_directory

        self._start_record_datetime = None
        self._audio_data = None

    def _record_callback(self, data):

        is_record = data.data
        self._record(is_record)

    def _record(self, is_record):

        if is_record:
            if self._is_recording():
                rospy.logerr("Already recording audio")
            else:
                rospy.loginfo("Starting to record audio")
                self._audio_data = []
                self._start_record_datetime = datetime.datetime.now()
        else:
            if self._is_recording():

                rospy.loginfo("Stopped recording audio")
                self._save_recording(self._audio_data)

                # Clean up
                self._audio_data = None
                self._start_record_datetime = None
            else:
                rospy.logerr("No recording in progress")

    def _audio_data_callback(self, data):
        if self._is_recording():
            self._audio_data.append(data.data)

    def _is_recording(self):
        return self._start_record_datetime is not None

    def _save_recording(self, audio_data):

        if not os.path.exists(self._out_directory):
            os.makedirs(self._out_directory)
        file_name = "{date_str}.{ext}".format(
            date_str=datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'),
            ext='wav'
        )
        file_path = os.path.join(self._out_directory, file_name)

        wf = wave.open(file_path, 'wb')
        wf.setnchannels(CHANNEL)
        wf.setsampwidth(pyaudio.PyAudio().get_sample_size(FORMAT_SIZE))
        wf.setframerate(SAMPLE_RATE)
        wf.setnframes(CHUNK_SIZE)
        wf.writeframes(b''.join(audio_data))
        wf.close()


if __name__ == "__main__":

    ac = AudioCapture(
        is_record_topic=IS_RECORD_TOPIC,
        audio_data_topic=AUDIO_DATA_TOPIC,
        out_file_directory=OUT_DIRECTORY,
    )
    rospy.spin()
