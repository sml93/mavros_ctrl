#!/usr/bin/env python

from __future__ import division
from __future__ import print_function
from __future__ import absolute_import


PKG = 'px4'

import math
import time
import rospy
import thread
import threading
import mavros
import numpy as np

from math import *
from mavros.utils import *
from six.moves import xrange
from threading import Thread
from pymavlink import mavutil
from std_msgs.msg import Bool
from std_msgs.msg import Header
from mavros import setpoint as SP
from mavros_msgs.msg import ParamValue
from mavros_test_common import MavrosTestCommon
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion


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

        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")

            # TODO: Clean this up.
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
        def is_near(msg, des, current):
            rospy.logdebug("Position %s: local: %.2f, target: %.2f, abs diff: %.3f",
                           msg, des, current, np.linalg.norm(des - current))
            return np.linalg.norm(des - current) < 0.5

        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()


class MavrosOffboardPosctlTest(MavrosTestCommon):
  def setUp(self):
    super(MavrosOffboardPosctlTest, self).setUp()
    setpoint = SetpointPosition()
    self.pos_thread = Thread(target=setpoint.navigate, args=())
    self.pos_thread.daemon = True
    self.pos_thread.start()
    
  def tearDown(self):
      super(MavrosOffboardPosctlTest, self).tearDown()


  def setpoint_demo(self):
      rospy.init_node('setpoint_position')
      mavros.set_namespace()
      rate = rospy.Rate(10)

      self.wait_for_topics(60)
      self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                 10, -1)
      self.log_topic_vars()
      rcl_except = ParamValue(1<<2, 0.0)
      self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
      self.set_mode("OFFBOARD", 5)
      self.set_arm(True, 5)

      time.sleep(1)

      setpoint = SetpointPosition()

      rospy.loginfo("Hover")
      setpoint.set(0.0, 0.0, 1.5, 5)
      setpoint.set(0.0, 0.0, 1.6, 2)

      rospy.loginfo("Landing")
      for k in range(101):
          set_x = 0.0
          set_y = 0.0
          set_z = 1.6 - (0.013*k)
          print(round(set_x,3), round(set_y,3), round(set_z,3))
          setpoint.set(set_x, set_y, set_z, 0)

      while not rospy.is_shutdown():
        if not self.mission_done.data:
          setpoint.set(0.0, 0.0, 0.3, 2)
        else:
          break

      self.set_mode("AUTO.LOITER", 5)
      t = 3
      while t:
        mins, secs = divmod(t, 60)
        timer = '{:02d:}:{:02d}'.format(mins, secs)
        rospy.loginfo("Timer: {:02d:}:{:02d}".format(mins, secs))
        time.sleep(1)
        t -= 1

      self.set_mode("AUTO.LAND", 5)
      self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
      self.set_arm(False, 5)


if __name__ == "__main__":
  import rostest
  rospy.init_node('test_node', anonymous=True)
  rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest)
