# Robotic Arm
A robotic arm using Robot Operating System based off of [EEZYbotARM](https://www.instructables.com/EEZYbotARM/), and using Alexa SDK for voice control. This project is using both C++ and Python.

![rviz](/images/rviz.png)

## Technologies
- Robot Operating System (Noetic)
- Rviz
- Gazebo
- MoveIt 
- NgRok
- Alexa SDK

## Structure
- arduinobot_description contains URDF for building the simulated rotbotic arm
- arduinobot_test contains scripts and cpp files used to learn the basics of ROS such as publisher/subscriber nodes, services, and timers.
- arduinobot_moveit contians the configuration files for ROS MovIT
- arduinobot_controller contains the controller interface and converter service
- arduinobot_remote contains the services to interact with the robot with Alexa voice control
- arduinobot_bringup contains the launch files for starting the project

## Demo

## Next Steps
Move to a pyshical setup and print out the components of the robot and use an Arduino board and servo motors for control.
``