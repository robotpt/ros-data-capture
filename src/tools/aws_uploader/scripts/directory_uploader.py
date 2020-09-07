#!/usr/bin/env python

"""
This script uploads all files that go into an specific directory to Amazon Web Services
S3 storage. It takes two arguements:
    1. the path to the directory that should be uploaded
    2. a name for the bucket used

Note that this script requires a few parameters before running. You can load them
through the commandline with `rosparam` or use the launch file.

Usage example (once parameters are loaded):

    python directory_uploader.py  /root/upload  my-amazon-bucket-1234512345

"""

import rospy
import sys
import os

from std_msgs.msg import String
from aws_uploader.aws_file_uploader import AwsFileUploader


DIR_IDX = 1
BUCKET_NAME_IDX = 2


def main():

    region = rospy.get_param('aws/region_name')

    is_remove_file_on_upload = rospy.get_param('uploader/is_remove_file_on_upload')
    is_create_directory_if_doesnt_exist = rospy.get_param(
        'uploader/is_create_directory_if_doesnt_exist')
    publish_topic = rospy.get_param('uploader/publish_topic')
    seconds_delay_to_check_if_file_is_being_written = rospy.get_param(
        'uploader/seconds_delay_to_check_if_file_is_being_written')
    args = sys.argv
    directory_path = args[DIR_IDX]
    bucket_name = args[BUCKET_NAME_IDX]

    if not os.path.exists(directory_path):
        if is_create_directory_if_doesnt_exist:
            os.makedirs(directory_path, exist_ok=True)
        else:
            raise IOError("Directory doesn't exist: '{}'".format(directory_path))
    if not os.path.isdir(directory_path):
        raise IOError("Path given is not to a directory: '{}'".format(directory_path))

    directory_name = _get_directory_name_using_ros_char_set(directory_path)
    rospy.init_node(
        'aws_uploader_{}'.format(directory_name),
        anonymous=True,
    )
    uploaded_publisher = rospy.Publisher(publish_topic, String, queue_size=1)

    aws_file_uploader = AwsFileUploader(
        region=region,
        default_bucket=bucket_name,
        seconds_delay_to_check_if_file_is_being_written=seconds_delay_to_check_if_file_is_being_written,
    )

    def upload_directory_callback(_):
        uploaded_files, not_uploaded_files = aws_file_uploader.upload_directory_contents_to_aws_directory(
            directory_path=directory_path,
            is_remove_file_on_upload=is_remove_file_on_upload,
        )
        if len(uploaded_files):
            rospy.loginfo("UPLOADED {}".format(uploaded_files))
            for f in uploaded_files:
                msg = String()
                msg.data = f
                uploaded_publisher.publish(msg)
        if len(not_uploaded_files):
            rospy.logdebug("The following are being written to: {}".format(not_uploaded_files))

    rospy.Timer(rospy.Duration(2), upload_directory_callback)
    rospy.spin()


def _get_directory_name_using_ros_char_set(directory_path):
    return os.path.basename(os.path.normpath(directory_path)).replace("-", "_")


if __name__ == '__main__':

    main()
