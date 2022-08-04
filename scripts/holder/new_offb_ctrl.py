#!/usr/bin/env python2.7

from __future__ import division
from __future__ import print_function
from __future__ import absolute_import


PKG = 'px4'

import math
import time
import rospy
import numpy as np

from six.moves import xrange
from threading import Thread
from pymavlink import mavutil
from std_msgs.msg import Bool
from std_msgs.msg import Header
from mavros_msgs.msg import ParamValue
from mavros_test_common import MavrosTestCommon
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion


class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.
    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.pos = PoseStamped()
        self.radius = 1

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        self.Bool = Bool(True)
        self.attRunning_pub = rospy.Publisher(
            'att_running_msg', Bool, queue_size=10)

        # # self.mission_done = Bool(False)
        # self.mission_status_sub = rospy.Subscriber(
        #     'mission_status', Bool, self.mission_status_cb)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    # def mission_status_cb(self, data):
    #   self.mission_done = data

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            self.attRunning_pub.publish(self.Bool)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        # start_timer = rospy.Time.now()

        time.sleep(1)

        rospy.loginfo("run mission")
        # positions = ((0, 0, 0), (10, 10, 5), (10, -10, 5), (-10, -10, 5),
        #              (0, 0, 5))
        # positions = ((0, 0, 5), (0, 0, 5), (0, 0, 10))

        rospy.loginfo("Mission done?: {0}".format(str(self.mission_done.data)))
        while not rospy.is_shutdown():
          if not self.mission_done.data:
            self.reach_position(0, 0, 10, 30)
          else:
            break
                  
        # time.sleep(15)
        self.set_mode("AUTO.LOITER", 5)
        t = 3
        while t:
            mins, secs = divmod(t, 60)
            timer = '{:02d}:{:02d}'.format(mins, secs)
            # print(timer, end="\r")
            rospy.loginfo("Timer: {:02d}:{:02d}".format(mins, secs))
            time.sleep(1)
            t -= 1
            

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
