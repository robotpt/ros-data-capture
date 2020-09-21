#!/usr/bin/env python

import rospy
import sys
import os

from std_msgs.msg import String

from directory_size_manager import DirectorySizeManager


DIR_IDX = 1


def main():

    is_create_directory_if_doesnt_exist = rospy.get_param(
        'directory_size_manager/is_create_directory_if_doesnt_exist',
    )
    max_size_in_bytes = 1024*rospy.get_param(
        'directory_size_manager/max_size_in_kilobytes',
    )
    publish_topic = rospy.get_param('directory_size_manager/publish_topic')

    args = sys.argv
    directory_path = args[DIR_IDX]

    if not os.path.exists(directory_path):
        if is_create_directory_if_doesnt_exist:
            os.makedirs(directory_path)
        else:
            raise IOError("Directory doesn't exist: '{}'".format(directory_path))
    if not os.path.isdir(directory_path):
        raise IOError("Path given is not to a directory: '{}'".format(directory_path))

    rospy.init_node(
        'directory_size_mananger_{}'.format(_get_directory_name_using_ros_char_set(directory_path)),
        anonymous=True,
    )
    deleter_publisher = rospy.Publisher(publish_topic, String, queue_size=1)

    dsm = DirectorySizeManager(directory=directory_path, max_size_in_bytes=max_size_in_bytes)

    def directory_manager_callback(_):
        while dsm.is_too_large():

            rospy.loginfo("Deleting file '{}'".format(dsm.oldest_file))

            msg = String()
            msg.data = dsm.oldest_file
            deleter_publisher.publish(msg)

            dsm.run_once()

    rospy.Timer(rospy.Duration(2), directory_manager_callback)
    rospy.spin()


def _get_directory_name_using_ros_char_set(directory_path):
    return os.path.basename(os.path.normpath(directory_path)).replace("-", "_")


if __name__ == '__main__':
    main()
