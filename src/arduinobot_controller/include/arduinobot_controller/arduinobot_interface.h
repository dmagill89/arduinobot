#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <vector>
#include <boost/scoped_ptr.hpp>

#pragma once

class ArduinobotInterface : public hardware_interface::RobotHW
{
    public:
        ArduinobotInterface(ros::NodeHandle&);
        void update(const ros::TimerEvent&);
        void read();
        void write(ros::Duration);

    private:
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        ros::Duration elapsedTime;
        ros::Duration updateFrequency;
        ros::Timer looper;
        ros::Publisher hardwarePublisher;
        ros::ServiceClient hardwareService;
        hardware_interface::JointStateInterface jointStateInterfrace;
        hardware_interface::PositionJointInterface jointPositionInterface;
        boost::shared_ptr<controller_manager::ControllerManager> controlleManager;
        std::vector<double> commands;
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;
        std::vector<std::string> names;
};