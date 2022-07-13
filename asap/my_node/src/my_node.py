#!/usr/bin/env python3
import rospy

from ff_msgs.msg import EkfState
from ff_msgs.msg import FamCommand
from std_msgs.msg import String
import numpy as np

VERBOSE = 1


class MyNode():
    def __init__(self):
        """
        Create ROS params, subs, pubs.
        """
        rospy.loginfo('[MyNode]: Initialized.')

        # ***Params***
        # set timing parameters
        rospy.set_param("/mynode/test1", 1)

        # tracking points
        self.POINT_A_GRANITE = rospy.get_param("/asap/primary/point_a_granite")
        self.POINT_A_ISS = rospy.get_param("/asap/primary/point_a_iss")
        self.POINT_B_GRANITE = rospy.get_param("/asap/primary/point_b_granite")
        self.POINT_B_ISS = rospy.get_param("/asap/primary/point_b_iss")
        self.POINT_C_GRANITE = rospy.get_param("/asap/primary/point_c_granite")
        self.POINT_C_ISS = rospy.get_param("/asap/primary/point_c_iss")

        # ***Subs***
        rospy.Subscriber("gnc/ekf", EkfState, self.ekf_state_callback)

        # ***Pubs***
        self.control_mode_pub_ = rospy.Publisher("asap/primary/control_mode", String, queue_size=3)  # for casadi_nmpc

        print("MyNode got params and subs! Continuing to spin...")
        print("At this point, MyNode will wait for any tests commands to get sent out from coordinator.")
        print("Check out coordinator/README.md.")

    def ekf_state_callback(self, msg):
        """ Update latest estimated robot pose.

        x_est_ = [r(3) v(3) q(4) w(3)].T
        """
        r = np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]]).T
        v = np.array([[msg.velocity.x, msg.velocity.y, msg.velocity.z]]).T
        q = np.array([[msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]]).T
        w = np.array([[msg.omega.x, msg.omega.y, msg.omega.z]]).T
        self.x_est_ = np.vstack((r, v, q, w))


if __name__ == '__main__':
    if VERBOSE == 1:
        rospy.init_node('my_node')
    else:
        rospy.init_node('my_node', log_level=rospy.ERROR)
    node = MyNode()
    while not rospy.is_shutdown():
        rospy.spin()
