#!/usr/bin/env python2

from __future__ import division

PKG = 'px4'

import rospy
import threading

from six.moves import xrange
from threading import Thread
from pymavlink import mavutil
from std_msgs.msg import Bool
from std_msgs.msg import Header
from mavros_msgs.msg import ParamValue, AttitudeTarget
from mavros_test_common import MavrosTestCommon
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3


class MavrosOffboardAttCtlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.
    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardAttCtlTest, self).setUp()

        self.pos = PoseStamped()
        self.att = AttitudeTarget()

        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        self.Bool = Bool(True)
        self.attRunning_pub = rospy.Publisher(
            'att_running_msg', Bool, queue_size=10)

        # # send setpoints in seperate thread to better prevent failsafe
        # self.pos_thread = Thread(target=self.send_pos, args=())
        # self.pos_thread.daemon = True
        # self.pos_thread.start()

        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()

        self.done = False
        self.done_evt = threading.Event()

    def tearDown(self):
        super(MavrosOffboardAttCtlTest, self).tearDown()


    #
    # Helper methods: for sending pos or atti
    #
    def send_att(self):
        rate = rospy.Rate(10)       # Hz
        self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        # self.att.body_rate = self.desired_atti.body_rate      # topic based
        self.att.body_rate = (0.1, 0.0, 0.1)
        # self.att.thrust = self.desired_atti.thrust            # topic based
        self.att.thrust = 0.7
        self.att.type_mask = 128 # ignore orientation field

        while not rospy.is_shutdown():
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    #
    # Test method 1: Att Ctrl
    #
    def test_attctrl(self):
        """ Test offboard atti control """
        boundary_x = 20
        boundary_y = 10
        boundary_z = 2

        # Ensure simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1<<2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        rospy.loginfo("attempting to cross boundary | x:{0}, y:{1}, z:{2}".
                       format(boundary_x, boundary_y, boundary_z))

        timeout = 90        # (int) secs
        loop_freq = 2
        rate = rospy.Rate(loop_freq)
        crossed = False
        rospy.loginfo("HERE: ", self.mission_done.data)
        rospy.loginfo("Mission done?: {0}".format(str(self.mission_done.data)))
        # while not rospy.is_shutdown():
        #     if not self.mission_done.data:
        for i in xrange(timeout * loop_freq):
            if (self.local_position.pose.position.x > boundary_x and
                self.local_position.pose.position.y > boundary_y and 
                self.local_position.pose.position.z > boundary_z):
                rospy.loginfo("Boundary crossed | seconds: {0} of {1}".
                            format(i/loop_freq, timeout))
                crossed = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(crossed, ("Took too long to cross boundaries | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(secs): {3}".
                                format(self.local_position.pose.position.x,
                                        self.local_position.pose.position.y,
                                        self.local_position.pose.position.z, timeout)))
        # self.send_att()
            # else: break
                  
        # self.set_mode("AUTO.LOITER", 5)
        # t = 3
        # while t:
        #     mins, secs = divmod(t, 60)
        #     timer = '{:02d}:{:02d}'.format(mins, secs)
        #     # print(timer, end="\r")
        #     rospy.loginfo("Timer: {:02d}:{:02d}".format(mins, secs))
        #     time.sleep(1)
        #     t -= 1
            

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_ctl_test',
                   MavrosOffboardAttCtlTest)
