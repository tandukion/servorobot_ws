#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
2019/10/08
@author: dwindra
"""

import unittest
import rospy
import threading
from std_msgs.msg import Empty, String

PKG = 'robot_motion'


class TestPickPlace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_pick_place')

        cls.robot_state = None
        # Event thread for waiting until robot ready
        cls.robot_state_event = threading.Event()
        cls.job_finish_event = threading.Event()

    def recv_state(self, msg):
        if msg.data:
            self.robot_state = msg.data
            self.robot_state_event.set()

    def recv_job_status(self, msg):
        if msg.data == "finished":
            self.job_finish_event.set()

    def test_pick_and_place(self):
        rospy.Subscriber('/state_machine/state', String, self.recv_state)
        rospy.Subscriber('/job_status', String, self.recv_job_status)
        self._start_pick_place = rospy.Publisher('/start_pick_place', Empty, queue_size=10)

        rospy.sleep(1)

        # Wait until the system is initialized, if it is still uninitialized
        if not self.robot_state == 'standby':
            rospy.loginfo ("Waiting until system is ready")
            self.robot_state_event.clear()
            self.robot_state_event.wait()
            rospy.sleep(1)

        rospy.loginfo ("Sending command to start the job")
        self.job_finish_event.clear()
        self._start_pick_place.publish()
        self.job_finish_event.wait()


if __name__ == '__main__':
    # unittest.main()
    import rosunit
    rosunit.unitrun(PKG, 'test_pick_place', TestPickPlace)
