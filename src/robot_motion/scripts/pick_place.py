#!/usr/bin/env python
"""
2019/10/08
@author : dwindra
"""

import rospy
import sys
import threading
import os
import yaml
import copy
import time
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
from std_msgs.msg import Empty, String
from transitions.extensions import GraphMachine


config_dir = '../config/'
state_machine_file = 'state_machine.yaml'
state_machine_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config_dir, state_machine_file)


class PickPlaceLogic(GraphMachine):
    GROUP_NAME = 'manipulator'

    def __init__(self):
        rospy.init_node('pick_place_logic')

        # initialize state machine
        # state_machine_config = rospy.get_param('/ipr/state_machine_config')
        # GraphMachine.__init__(self, title='Pick and Place State Manager', **state_machine_config)

        file_stream = open(state_machine_file_path, 'r')
        state_machine_config = yaml.load(file_stream)
        GraphMachine.__init__(self, title='Pick and Place State Manager',
                              states=state_machine_config['states'],
                              transitions=state_machine_config['transitions'],
                              initial=state_machine_config['initial'])

        # Moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self._group = moveit_commander.MoveGroupCommander(self.GROUP_NAME)
        self._robot = moveit_commander.RobotCommander()
        self._motion_result_code = None

        # Subscriber
        rospy.Subscriber('/execute_trajectory/result', moveit_msgs.msg.ExecuteTrajectoryActionResult, self._motion_result_callback)
        rospy.Subscriber('/start_pick_place', Empty, self.start_pick_and_place)

        # Publisher
        self._state_machine_pub = rospy.Publisher('/state_machine/state', String, queue_size=10)
        self._job_status_pub = rospy.Publisher('/job_status', String, queue_size=10)

        # Event thread
        self._motion_result_event = threading.Event()

        rospy.loginfo('Robot is initialized')
        self.trig_initialized()

    def _motion_result_callback(self, msg):
        self._motion_result_code = msg.result.error_code.val
        self._motion_result_event.set()

    def _execute_motion(self, motion):
        self._group.execute(motion, wait=False)
        self._motion_result_event.wait()
        if self._motion_result_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            rospy.loginfo('Motion was successful')
        else:
            rospy.loginfo('Motion was unsuccessful')

    def start_pick_and_place(self, msg):
        self.trig_pick()

    # STATE MACHINE callbacks
    def on_enter_initialized(self):
        rospy.sleep(1)
        self.trig_go_home()

    def on_enter_go_home(self):
        rospy.loginfo('Going home')

        # Set home position
        waypoints = []
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = 0.5
        wpose.position.y = 0
        wpose.position.z = 2.5
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self._group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.1,        # eef_step
                                           0.0)         # jump_threshold

        if plan:
            self._execute_motion(plan)
            self.trig_standby()

    def on_enter_pick(self):
        rospy.loginfo('Entering pick motion')
        rospy.sleep(1)
        self.trig_place()

    def on_enter_place(self):
        rospy.loginfo('Entering place motion')
        rospy.sleep(1)

        job_finish = True
        if job_finish:
            self._job_status_pub.publish('finished')
            self.trig_go_home()

    def on_enter_standby(self):
        rospy.loginfo('Robot is STANDBY')
        t = threading.Thread(target=self._publish_robot_state)
        t.setDaemon(True)
        t.start()

    def _publish_robot_state(self):
        while self.state == 'standby':
            self._state_machine_pub.publish(self.state)
            time.sleep(0.02)

if __name__ == '__main__':
    try:
        logic = PickPlaceLogic()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
