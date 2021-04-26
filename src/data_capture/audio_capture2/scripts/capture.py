#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from audio_common_msgs.msg import AudioData

import os
import argparse

import pyaudio
import wave
import datetime


class AudioCapture:

    def __init__(
            self,
            is_record_topic,
            audio_data_topic,
            num_channels,
            sample_rate,
            chunk_size,
            format_size,
            file_name_prefix='',
            out_file_directory='audio',
    ):

        rospy.init_node('audio_recorder', anonymous=True)

        self._is_record_subscriber = rospy.Subscriber(is_record_topic, Bool, self._record_callback, queue_size=1)
        self._audio_data_topic = rospy.Subscriber(audio_data_topic, AudioData, self._audio_data_callback, queue_size=1)

        self._num_channels = num_channels
        self._sample_rate = sample_rate
        self._chunk_size = chunk_size
        self._format_size = format_size
        self._file_name_prefix = file_name_prefix
        if len(self._file_name_prefix) > 0:
            self._file_name_prefix += '_'
        rospy.loginfo(self._file_name_prefix)
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
        file_name = "{prefix}{date_str}.{ext}".format(
            prefix=self._file_name_prefix,
            date_str=datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'),
            ext='wav'
        )
        file_path = os.path.join(self._out_directory, file_name)

        wf = wave.open(file_path, 'wb')
        wf.setnchannels(self._num_channels)
        wf.setsampwidth(pyaudio.PyAudio().get_sample_size(self._format_size))
        wf.setframerate(self._sample_rate)
        wf.setnframes(self._chunk_size)
        wf.writeframes(b''.join(audio_data))
        wf.close()


if __name__ == "__main__":

    # Having the topic as a command line argument allows multiple video recorder nodes to exist on different topics
    parser = argparse.ArgumentParser(description='Record video from an image topic')
    parser.add_argument('--audio-topic', help='The audio topic for the program to subscribe to',
                        default="audio/audio")
    parser.add_argument('--is-record-topic', help='Topic that publishes if recordings should start or stop',
                        default='audio_capture/is_record')
    parser.add_argument('--output-directory', help='Directory where audio should be saved to',
                        default="/root/audio")
    parser.add_argument('--num-channels', type=int, help='Number of audio channels',
                        default=1)
    parser.add_argument('--chunk-size', type=int, help='The size in bits of audio chunks received',
                        default=1024)
    parser.add_argument('--sample-rate', type=int, help='The sample rate that the audio is recorded in',
                        default=16000)
    parser.add_argument('--format', help='The format of the audio received (currently only supports WAV)',
                        default="wave")
    parser.add_argument('--format-size', help='The format encoding used by PyAudio',
                        default=pyaudio.paInt16)
    parser.add_argument('--file-name-prefix',
                        type=str,
                        help='The string to prepend to the file name; for example, \'evaluation\'',
                        default='')

    args, _ = parser.parse_known_args()

    assert args.format == "wave"

    AudioCapture(
        is_record_topic=args.is_record_topic,
        audio_data_topic=args.audio_topic,
        out_file_directory=args.output_directory,
        num_channels=args.num_channels,
        sample_rate=args.sample_rate,
        chunk_size=args.chunk_size,
        format_size=args.format_size,
        file_name_prefix=args.file_name_prefix
    )
    rospy.spin()
