#!/usr/bin/env pyhthon3
import rospy
import actionlib
from arduinobot_test.msg import FibonacciAction, FibonacciGoal

def fibonacci_client():
    # Create a SimpleAction client
    client = actionlib.SimpleActionClient("fibonacci", FibonacciAction)
    
    # Waits until the action server has started up and began listening
    client.wait_for_server()

    # Creates a goal to send to action server
    goal = FibonacciGoal(goal=20)
   
    # send goal 
    print("sending goal order %s" % goal.goal)
    client.send_goal(goal)
    client.wait_for_result()
    
    return client.get_result()


if __name__ == '__main__':
    # Initialize a ROS node called fibonacci_client
    rospy.init_node("fibonacci_client")
    rospy.loginfo("simple action client started")

    # Keep the node active until the action server returns a result
    result = fibonacci_client()

    print("Result: ", result.sequence)