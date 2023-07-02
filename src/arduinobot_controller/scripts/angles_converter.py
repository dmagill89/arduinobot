#!/usr/bin/env python3
import rospy
from arduinobot_controller.srv import AnglesConverter, AnglesConverterResponse
import math

"""
Function call back for radians_to_degrees service. This function receives the position for the
arm in radians and converts them to degrees.
"""
def convert_radians_to_degrees(req):
    res = AnglesConverterResponse()
    res.base = int(((req.base+(math.pi/2))*180)/math.pi)
    res.shoulder = 180-int(((req.shoulder+(math.pi/2))*180)/math.pi)
    res.elbow = int(((req.elbow+(math.pi/2))*180)/math.pi)
    res.gripper = int(((-req.gripper)*180)/(math.pi/2))

    return res

"""
Function call back for degrees_to_radians service. This function receives the position for the
arm in degrees and converts them to radians.
"""
def convert_degrees_to_radians(req):
    res = AnglesConverterResponse()
    res.base = ((math.pi*req.base) - ((math.pi/2)*180))/180
    res.shoulder = (((180-req.shoulder)*math.pi)-((math.pi/2)*180))/180
    res.elbow = ((math.pi*req.elbow) - ((math.pi/2)*180))/180
    res.gripper = -((math.pi/2)*req.gripper)/180

    return res

if __name__ == '__main__':
    # Initialize a ROS node names angles_converter
    rospy.init_node("angles_converter")

    # Initialize a service to convert radians to degrees and vice versa
    radians_to_degress = rospy.Service("radians_to_degress", AnglesConverter, convert_radians_to_degrees)
    degrees_to_radians = rospy.Service("degress_to_radians", AnglesConverter, convert_degrees_to_radians)
    
    # keeps the node up and running
    rospy.spin()

