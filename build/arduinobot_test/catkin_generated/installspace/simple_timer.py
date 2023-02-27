#!/usr/bin/env python3
import rospy

def timer_callback(event=None):
     rospy.loginfo("Called timer callback function")

if __name__ == '__main__':
    rospy.init_node('timer_node', anonymous=True)
    timer_duration = rospy.Duration(1)
    rospy.Timer(timer_duration, timer_callback)

    rospy.spin()