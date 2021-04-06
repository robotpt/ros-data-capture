#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import psutil
import argparse


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


def ram_usage_watcher(mem_upper_limit):
    pub = rospy.Publisher(
        'data_capture/is_memory_usage_exceeded', Bool, queue_size=1)
    rospy.init_node('ram_usage_watcher', anonymous=True)
    rate = rospy.Rate(0.2)  # Once every 5 seconds = 1/5 = 0.2 hz
    while not rospy.is_shutdown():

        # Check on free memory if exceeds 90% utilization
        mem_usage = monitorAvailableMemory(
            memory_upperlimit_percent=mem_upper_limit)
        pub.publish(mem_usage)
        rate.sleep()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument('--mem-upper-limit', type=float, help='Memory utilization upper limit in percent',
                        default=90.0)

    args, _ = parser.parse_known_args()

    try:
        ram_usage_watcher(args.mem_upper_limit)
    except rospy.ROSInterruptException:
        pass
