#!/usr/env/bin python3
import rospy
import actionlib
from arduinobot_test.msg import FibonacciAction, FibonacciResult, FibonacciFeedback

class FibonacciActionServer(object):
    feedback_ = FibonacciFeedback()
    result_ = FibonacciResult()

    def __init__(self, name):
        self.action_name = name
        self.as_ = actionlib.SimpleActionServer(self.action_name, FibonacciAction, execute_cb = self.execute_cb, auto_start = False)
        self.as_.start()
        rospy.loginfo('Simple Action Server Started')

    
    def execute_cb(self, goal):
        rospy.loginfo('Goal Received %s' % goal.goal)
        r = rospy.Rate(1)
        success = True
        self.feedback_.sequence = []
        self.feedback_.sequence.append(1)
        self.feedback_.sequence.append(1)

        for i in range(1, goal.goal - 1):
            if self.as_.is_preempt_requested():
                rospy.loginfo('%s Preempted' % self.action_name)
                self.as_.set_preempted()
                success = False
                break

            self.feedback_.sequence.append(self.feedback_.sequence[i] + self.feedback_.sequence[i -1])
            self.as_.publish_feedback(self.feedback_)
            r.sleep()

        if success:
            self.result_.sequence = self.feedback_.sequence
            rospy.loginfo('%s Succeeded' % self.action_name)
            self.as_.set_succeeded(self.result_)


if __name__ == '__main__':
    rospy.init_node('fibonacci')
    
    server = FibonacciActionServer('fibonacci')
    rospy.spin()