#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import psutil


def monitorAvailableMemory(memory_upperlimit_percent):
    """
    This function is used to monitor the memory utilization and throw an error 
    if it exceeds a preset value.

    Arguments:
    memory_upperlimit_percent: The upperlimit of the memory utilization (float)
    """

    # Utilized memory
    utilized_memory = psutil.virtual_memory().percent

    if utilized_memory > memory_upperlimit_percent:
        return True

    return False


def mem_use_watcher():
    pub = rospy.Publisher('is_memory_usage_exceeded', Bool, queue_size=10)
    rospy.init_node('mem_use_watcher', anonymous=True)
    rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():

        # Check on free memory if exceeds 90% utilization
        mem_usage = monitorAvailableMemory(memory_upperlimit_percent=90)
        pub.publish(mem_usage)
        rate.sleep()


if __name__ == '__main__':
    try:
        mem_use_watcher()
    except rospy.ROSInterruptException:
        pass
