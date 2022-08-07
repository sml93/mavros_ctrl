#!/usr/bin/env python2

from __future__ import division

PKG = 'px4'

import time
import rospy
import threading

from threading import Thread
from six.moves import xrange
from pymavlink import mavutil
from std_msgs.msg import Header, Bool
from mavros_msgs.msg import AttitudeTarget
from mavros_test_common import MavrosTestCommon
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import quaternion_from_euler


class MavrosOffboardAttctlTest(MavrosTestCommon):
    """
    Tests flying in offboard control by sending attitude and thrust setpoints
    via MAVROS.

    For the test to be successful it needs to cross a certain boundary in time.
    """

    def setUp(self):
        super(MavrosOffboardAttctlTest, self).setUp()

        self.att = AttitudeTarget()

        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        self.Bool = Bool(True)
        self.attRunning_pub = rospy.Publisher(
            'att_running_msg', Bool, queue_size=10)

        # send setpoints in seperate thread to better prevent failsafe
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

        self.done = False
        self.done_evt = threading.Event()

    def tearDown(self):
        super(MavrosOffboardAttctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_att(self):
        rate = rospy.Rate(10)  # Hz
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.type_mask = 128  # ignore orientation
        # self.att.orientation = Quaternion(*quaternion_from_euler(-0.25, 0.15, 0))
        self.att.body_rate.x = 0.0      # roll
        self.att.body_rate.y = 0.1      # pitch 
        self.att.body_rate.z = 0.0      # yaw
        self.att.thrust = 0.75
        

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            self.attRunning_pub.publish(self.Bool)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    #
    # Test method
    #
    def test_attctl(self):
        """Test offboard attitude control"""
        # boundary to cross
        boundary_x = 20
        boundary_y = 10
        boundary_z = 2

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        rospy.loginfo("attempting to cross boundary | x: {0}, y: {1}, z: {2}".
                      format(boundary_x, boundary_y, boundary_z))
        # does it cross expected boundaries in 'timeout' seconds?
        timeout = 90  # (int) seconds
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        crossed = False
        rospy.loginfo("Mission done?: {0}".format(str(self.mission_done.data)))
        for i in xrange(timeout * loop_freq):
            if (self.local_position.pose.position.x > boundary_x and
                    self.local_position.pose.position.y > boundary_y and
                    self.local_position.pose.position.z > boundary_z):
                rospy.loginfo("boundary crossed | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                crossed = True
                break
    
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(crossed, (
            "took too long to cross boundaries | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                self.local_position.pose.position.y,
                self.local_position.pose.position.z, timeout)))    

        self.send_att(0.0, -0.1, 0.0) 
        
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
                                   90, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_attctl_test',
                   MavrosOffboardAttctlTest)
