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
from industrial_msgs.msg import RobotStatus


config_dir = '../config/'
state_machine_file = 'state_machine.yaml'
state_machine_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config_dir, state_machine_file)

# pre-defined Pose
HOME = geometry_msgs.msg.Pose()
HOME.position.x = 1
HOME.position.y = 0
HOME.position.z = 2.25
HOME.orientation.w = 1
# HOME.position.x = 0.707
# HOME.position.y = 0
# HOME.position.z = 1.5
# HOME.orientation.w = 0.707
# HOME.orientation.y = 0.707

PREPICK = geometry_msgs.msg.Pose()
PREPICK.position.x = 0.707
PREPICK.position.y = 0.707
PREPICK.position.z = 1.5
PREPICK.orientation.w = 0.707
PREPICK.orientation.y = 0.707

PICK = geometry_msgs.msg.Pose()
PICK.position.x = 1
PICK.position.y = 1
PICK.position.z = 0
PICK.orientation.w = 0.707
PICK.orientation.y = 0.707

PREPLACE = geometry_msgs.msg.Pose()
PREPLACE.position.x = 0.707
PREPLACE.position.y = -0.707
PREPLACE.position.z = 1.5
PREPLACE.orientation.w = 0.707
PREPLACE.orientation.y = 0.707

PLACE = geometry_msgs.msg.Pose()
PLACE.position.x = 1
PLACE.position.y = -1
PLACE.position.z = 0
PLACE.orientation.w = 0.707
PLACE.orientation.y = 0.707

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

        self.home_pos = HOME
        # self.home_pos = self._group.get_current_pose().pose
        self.prepick_pos = PREPICK
        self.pick_pos = PICK
        self.preplace_pos = PREPLACE
        self.place_pos = PLACE
        self.waypoints = []
        self.job_finish = False

        # Subscriber
        rospy.Subscriber('/robot_status', RobotStatus, self._get_robot_state)
        rospy.Subscriber('/execute_trajectory/result', moveit_msgs.msg.ExecuteTrajectoryActionResult, self._motion_result_callback)
        rospy.Subscriber('/start_pick_place', Empty, self.start_pick_and_place)

        # Publisher
        self._state_machine_pub = rospy.Publisher('/state_machine/state', String, queue_size=10)
        self._job_status_pub = rospy.Publisher('/job_status', String, queue_size=10)

        # Event thread
        self._robot_status_event = threading.Event()
        self._motion_result_event = threading.Event()

        self._robot_status_event.wait()
        rospy.loginfo('Robot is initialized')
        self.trig_initialized()

    def _get_robot_state(self, robot_status):
        rospy.loginfo('Waiting for robot status')
        self._robot_mode = robot_status.mode.val
        self._robot_emergency_stopped = robot_status.e_stopped.val
        self._robot_servo_on = robot_status.drives_powered.val
        self._robot_status_event.set()

    def _motion_result_callback(self, msg):
        """
        Callback for motion result
        :param msg: message received
        """
        self._motion_result_code = msg.result.error_code.val
        self._motion_result_event.set()

    def _execute_motion(self, motion):
        self._group.execute(motion, wait=False)
        self._motion_result_event.wait()
        if self._motion_result_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            rospy.loginfo('Motion was successful')
        else:
            rospy.loginfo('Motion was unsuccessful')

        self._motion_result_event.clear()
        self.waypoints[:] = []

    def start_pick_and_place(self, msg):
        self.trig_pick()

    # STATE MACHINE callbacks
    def on_enter_initialized(self):
        rospy.sleep(1)
        self.trig_go_home()

    def on_enter_go_home(self):
        rospy.loginfo('Going home')

        # Set home position
        self.waypoints.append(copy.deepcopy(self.home_pos))

        # eef_step = 0.1, jump_threshold = 0.0 (disabled)
        (plan, fraction) = self._group.compute_cartesian_path(self.waypoints, 0.1, 0.0)

        if plan:
            self._execute_motion(plan)
            self.trig_standby()

    def on_enter_pick(self):
        rospy.loginfo('Entering pick motion')
        self.waypoints.append(copy.deepcopy(self.prepick_pos))
        self.waypoints.append(copy.deepcopy(self.pick_pos))

        # eef_step = 0.1, jump_threshold = 0.0 (disabled)
        (plan, fraction) = self._group.compute_cartesian_path(self.waypoints, 0.1, 0.0)
        if plan:
            self._execute_motion(plan)

        rospy.sleep(1)
        self.waypoints.append(copy.deepcopy(self.prepick_pos))
        self.trig_place()

    def on_enter_place(self):
        rospy.loginfo('Entering place motion')
        self.waypoints.append(copy.deepcopy(self.preplace_pos))
        self.waypoints.append(copy.deepcopy(self.place_pos))

        # eef_step = 0.1, jump_threshold = 0.0 (disabled)
        (plan, fraction) = self._group.compute_cartesian_path(self.waypoints, 0.2, 0.0)
        if plan:
            self._execute_motion(plan)

        rospy.sleep(1)
        self.waypoints.append(copy.deepcopy(self.preplace_pos))
        if self._motion_result_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            self.job_finish = True

        if self.job_finish:
            self.trig_go_home()

    def on_enter_standby(self):
        rospy.loginfo('Robot is STANDBY')
        # restart job finish status
        if self.job_finish:
            self._job_status_pub.publish('finished')
            self.job_finish = False
            rospy.sleep(1)

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
