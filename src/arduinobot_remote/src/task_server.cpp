#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "arduinobot_remote/ArduinobotTaskAction.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

class TaskServer
{
    private:
        ros::NodeHandle nodeHandle;
        actionlib::SimpleActionServer<arduinobot_remote::ArduinobotTaskAction> actionServer;
        std::string actionName;
        arduinobot_remote::ArduinobotTaskResult result;
        std::vector<double> armGoal;
        std::vector<double> gripperGoal;
        moveit::planning_interface::MoveGroupInterface armMoveGroup;
        moveit::planning_interface::MoveGroupInterface gripperMoveGroup;

    public:
        TaskServer(std::string name) :
                  actionServer(nodeHandle, name, boost::bind(&TaskServer::executeCb, this, _1), false),
                  actionName(name),
                  armMoveGroup("arduinobot_arm"),
                  gripperMoveGroup("arduinobot_hand")
        {
            actionServer.start();
        }

        void executeCb(const arduinobot_remote::ArduinobotTaskGoalConstPtr &goal) 
        {
            bool success = true;

            if (goal->task_number == 0)
            {
                armGoal = {0.0, 0.0, 0.0};
                gripperGoal = {-0.7, 0.7};
            }
            else if (goal->task_number == 1)
            {
                armGoal = {-1.14, -0.6, -0.07};
                gripperGoal = {0.0, 0.0};
            }
            else if (goal->task_number ==2)
            {
                armGoal = {-1.57, 0.0, -1.0};
                gripperGoal = {0.0, 0.0};
            }
            else 
            {
                ROS_ERROR("Invalid goal");
                return;
            }

            armMoveGroup.setJointValueTarget(armGoal);
            gripperMoveGroup.setJointValueTarget(gripperGoal);

            armMoveGroup.move();
            gripperMoveGroup.move();

            armMoveGroup.stop();
            gripperMoveGroup.stop();

            if (actionServer.isPreemptRequested() || ros::ok())
            {
                ROS_INFO("%s", actionName.c_str());
                actionServer.setPreempted();
                success = false;
            }

            if (success) 
            {
                result.success = true;
                ROS_INFO("%s Succeeded", actionName.c_str());
                actionServer.setSucceeded();
            }
        }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_server");
    TaskServer server("task_server");
    ros::spin();
    return 0;
}