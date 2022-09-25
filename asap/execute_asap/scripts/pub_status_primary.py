#!/usr/bin/env python3
"""
Publish the asap/status msg for debugging.
"""

import rospy
from coordinator.msg import StatusPrimary
import time


def pub_status():
    msg = StatusPrimary()
    msg.stamp = rospy.get_rostime()
    msg.test_number = 0
    msg.coord_ok = True
    msg.default_control = False
    msg.control_mode = "debug"
    msg.flight_mode = "nominal"

    time.sleep(0.5)
    print('msg sent...')
    pub = rospy.Publisher('/asap/status', StatusPrimary, queue_size=10, latch=True)
    pub.publish(msg)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pub_node')
    pub_status()
