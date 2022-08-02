#!/usr/bin/env python

import time
import rospy
import mavros
import thread
import threading
import numpy as np

from math import *
from mavros.utils import *
from mavros import setpoint as SP
# from tf.transformations import quaternion_from_euler
from std_msgs.msg import UInt16
from transforms3d.euler import euler2quat as quaternion_from_euler


class SetpointPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # self.jet = np.int16(0)

        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_position_local(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    SP.PoseStamped, self.reached)
        # self.jetpub = rospy.Publisher('servo', UInt16, queue_size=10)

        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")

            # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()

    def navigate(self):
        rate = rospy.Rate(10)   # 10hz

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

        while not rospy.is_shutdown():
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z

            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = SP.Quaternion(*quaternion)

            self.pub.publish(msg)
            rate.sleep()

    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z

        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()

        time.sleep(delay)

    def reached(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < 0.5

        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()

    # def jet(self, signal=0):
    #     self.jet = np.int16(signal)
    #     self.jetpub.publish(self.jet)


def setpoint_demo():
    rospy.init_node('setpoint_position')
    mavros.set_namespace()
    rate = rospy.Rate(10)
    jetpub = rospy.Publisher('servo', UInt16, queue_size=10)

    setpoint = SetpointPosition()

    rospy.loginfo("Climb")
    # for i in range(101):
    #     set_z = 0.01*i
    #     print("z: ", round(set_z,3))
    #     setpoint.set(0.0, 0.0, set_z, 0)
    # setpoint.set(0.0, 0.0, 1.0, 2)
    setpoint.set(0.0, 0.0, 1.3, 5)
    jetswitch = np.int16(0)
    jetpub.publish(jetswitch)

    rospy.loginfo("Getting into position")
    for j in range(11):
        set_x = 0.2*j
        set_y = 0.05*j
        set_z = 1.3
        print(set_x, set_y, set_z)
        setpoint.set(set_x, set_y, set_z, 0)

    rospy.loginfo("In position..wait for 5 secs")
    time.sleep(1)
    time.sleep(1)

    for i in range(3):
        count = 3 - i
        print('jet in', count)
    print("Jet now")
    # start_time = time.time()
    ## TODO: To add in jetting command
    jetswitch = np.int16(180)
    jetpub.publish(jetswitch)
    time.sleep(2)

    setpoint.set(2.0 + 0.5, 0.5, 1.5 - 0.1, 0)
    time.sleep(5)
    setpoint.set(2.0, 0.5, 1.5, 0)

    print("Stop jetting now")
    jetswitch = np.int16(0)
    jetpub.publish(jetswitch)

    rospy.loginfo("Back to home")
    for k in range(11):
        set_x = 2.0 - (0.2*k)
        set_y = 0.5 - (0.05*k)
        set_z = 1.5
        print(round(set_x,3), round(set_y,3), round(set_z,3))
        setpoint.set(set_x, set_y, set_z, 0)

    # rospy.loginfo("Getting into position")
    # setpoint.set(0.5, 0.5, 1.5, 2)
    # setpoint.set(1.0, 0.5, 1.5, 2)
    # setpoint.set(1.5, 0.5, 1.5, 2)
    # setpoint.set(2.0, 0.5, 1.5, 2)

    # rospy.loginfo("Back to home")
    # setpoint.set(1.5, 0.5, 1.5, 2)
    # setpoint.set(1.0, 0.5, 1.5, 2)
    # setpoint.set(0.5, 0.5, 1.5, 2)
    # setpoint.set(0.0, 0.0, 1.5, 2)

    rospy.loginfo("Landing")
    # Simulate a slow landing.

    # setpoint.set(0.0, 0.0,  1.0, 2)
    # setpoint.set(0.0, 0.0,  0.8, 2)
    # setpoint.set(0.0, 0.0,  0.5, 2)
    # setpoint.set(0.0, 0.0,  0.3, 2)
    # setpoint.set(0.0, 0.0,  0.2, 2)
    # setpoint.set(0.0, 0.0,  0.0, 2)
    for i in range(121):
        set_z = 1.5-(0.01*i)
        print("z: ", round(set_z,3))
        setpoint.set(0.0, 0.0, set_z, 0)
    # setpoint.set(0.0, 0.0, -0.2, 2)
    while not rospy.is_shutdown():
        setpoint.set(0.0, 0.0, 0.3, 2)


if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass
