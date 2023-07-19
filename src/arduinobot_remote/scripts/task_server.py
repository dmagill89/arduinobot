#!/usr/bin/env python3
import rospy
import actionlib
from arduinobot_remote.msg import ArduinobotTaskAction, ArduinobotTaskResult
import sys
import moveit_commander


class TaskServer(object):
    result = ArduinobotTaskResult()
    arm_goal = []
    gripper_goal = []

    def __init__(self, name):
        self.action_name_ = name
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_move_group_ = moveit_commander.MoveGroupCommander("arduinobot_arm")
        self.gripper_move_group_ = moveit_commander.MoveGroupCommander("arduinobot_hand")
        self.as_ = actionlib.SimpleActionServer(self.action_name_, ArduinobotTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self.as_.start()

    def execute_cb(self, goal):
        success = True

        if goal.task_number == 0:
            self.arm_goal = [0.0, 0.0, 0.0]
            self.gripper_goal = [-0.7, -0.7]
        elif goal.task_number == 1:
            self.arm_goal = [-1.14, -0.6, -0.07]
            self.gripper_goal = [0.0, 0.0]
        elif goal.task_number == 2:
            self.arm_goal = [-1.57, 0.0, -1.0]
            self.gripper_goal = [0.0, 0.0]
        else:
            rospy.logerr("Invalid goal")
            return
        
        self.arm_move_group_.go(self.arm_goal, wait=True)
        self.gripper_move_group_.go(self.gripper_goal, wait=True)

        self.arm_move_group_.stop()
        self.gripper_move_group_.stop()

        if self.as_.is_preempt_requested():
            rospy.loginfo("%s is Preempted" % self.action_name_)
            self.as_.set_preempted()
            success = False
        
        if success:
            self.result.success = True
            rospy.loginfo("%s Succeeded" % self.action_name_)
            self.as_.set_succeeded(self.result)


if __name__ == "__main__":
    rospy.init_node("task_server")
    server = TaskServer('task_server')
    rospy.spin()

        