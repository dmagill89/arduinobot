#include "arduinobot_controller/arduinobot_interface.h"
#include <std_msgs/UInt16MultiArray.h>
#include "arduinobot_controller/AnglesConverter.h"
#include <ros/ros.h>

ArduinobotInterface::ArduinobotInterface(ros::NodeHandle &nh) : nh(nh),
                                                                pnh("~"),
                                                                position(4, 0),
                                                                velocity(4, 0),
                                                                effort(4, 0),
                                                                commands(4, 0),
                                                                names{"joint_1", "joint_2", "joint_3", "joint_4"}
{
    hardwarePublisher = pnh.advertise<std_msgs::UInt16MultiArray>("/arduino/arm_actuate", 1000);
    hardwareService = pnh.serviceClient<arduinobot_controller::AnglesConverter>("/radians_to_degrees");

    hardware_interface::JointStateHandle stateHandle1(names.at(0), &position.at(0), &velocity.at(0), &effort.at(0));
    jointStateInterfrace.registerHandle(stateHandle1);
    hardware_interface::JointStateHandle stateHandle2(names.at(1), &position.at(1), &velocity.at(1), &effort.at(1));
    jointStateInterfrace.registerHandle(stateHandle2);
    hardware_interface::JointStateHandle stateHandle3(names.at(2), &position.at(2), &velocity.at(2), &effort.at(2));
    jointStateInterfrace.registerHandle(stateHandle3);
    hardware_interface::JointStateHandle stateHandle4(names.at(3), &position.at(3), &velocity.at(3), &effort.at(3));
    jointStateInterfrace.registerHandle(stateHandle4);

    registerInterface(&jointStateInterfrace);

    hardware_interface::JointHandle positionHandle1(jointPositionInterface.getHandle(names.at(0)), &commands.at(0));
    jointPositionInterface.registerHandle(positionHandle1);
    hardware_interface::JointHandle positionHandle2(jointPositionInterface.getHandle(names.at(1)), &commands.at(1));
    jointPositionInterface.registerHandle(positionHandle2);
    hardware_interface::JointHandle positionHandle3(jointPositionInterface.getHandle(names.at(2)), &commands.at(2));
    jointPositionInterface.registerHandle(positionHandle3);
    hardware_interface::JointHandle positionHandle4(jointPositionInterface.getHandle(names.at(3)), &commands.at(3));
    jointPositionInterface.registerHandle(positionHandle4);

    registerInterface(&jointPositionInterface);
    controlleManager.reset(new controller_manager::ControllerManager(this, nh));

    updateFrequency = ros::Duration(0.1);
    looper = nh.createTimer(updateFrequency, &ArduinobotInterface::update, this);
}

void ArduinobotInterface::update(const ros::TimerEvent &e)
{
    elapsedTime = ros::Duration(e.current_real - e.last_real);
    read();
    controlleManager->update(ros::Time::now(), elapsedTime);
    write(elapsedTime);
}

void ArduinobotInterface::read()
{
    position.at(0) = commands.at(0);
    position.at(1) = commands.at(1);
    position.at(2) = commands.at(2);
    position.at(3) = commands.at(3);
}

void ArduinobotInterface::write(ros::Duration elapsedTime)
{
    arduinobot_controller::AnglesConverter srv;
    srv.request.base = commands.at(0);
    srv.request.shoulder = commands.at(1);
    srv.request.elbow = commands.at(2);
    srv.request.gripper = commands.at(3);

    if (hardwareService.call(srv))
    {
        std::vector<unsigned int> anglesDegs;
        anglesDegs.push_back(srv.response.base);
        anglesDegs.push_back(srv.response.shoulder);
        anglesDegs.push_back(srv.response.elbow);
        anglesDegs.push_back(srv.response.gripper);

        std_msgs::UInt16MultiArray message;
        message.layout.dim.push_back(std_msgs::MultiArrayDimension());
        message.layout.dim[0].size = anglesDegs.size();
        message.layout.dim[0].stride = 1;
        message.data.clear();
        message.data.insert(message.data.end(), anglesDegs.begin(), anglesDegs.end());

        hardwarePublisher.publish(message);
    }
    else
    {
        ROS_ERROR("Failed to call radians_to_degress service");
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "arduinobot_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    ArduinobotInterface robot(nh);

    spinner.spin();
    return 0;
}