#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "arduinobot_test/FibonacciAction.h"

class FibonacciActionServer
{
    public:
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<arduinobot_test::FibonacciAction> actionServer;
        std::string actionName;
        arduinobot_test::FibonacciFeedback feedback;
        arduinobot_test::FibonacciResult result;

        void executeCB(const arduinobot_test::FibonacciGoalConstPtr &goal)
        {
            ROS_INFO("Goal received %i", goal->goal);
            ros::Rate rate(1);
            bool success = true;
            feedback.sequence.clear();
            feedback.sequence.push_back(1);
            feedback.sequence.push_back(1);

            for (int i = 1; i < goal->goal - 1; i++)
            {
                if (actionServer.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s is preempted", actionName.c_str());
                    actionServer.setPreempted();
                    success = false;
                    break;
                }

                feedback.sequence.push_back(feedback.sequence[i] + feedback.sequence[i - 1]);
                actionServer.publishFeedback(feedback);
                rate.sleep();
            }

            if (success)
            {
                result.sequence = feedback.sequence;
                ROS_INFO("%s has succeeded", actionName.c_str());
                actionServer.setSucceeded(result);
            }
        }

        FibonacciActionServer(std::string name) : actionServer(nh, name, boost::bind(&FibonacciActionServer::executeCB, this, _1), false),
                                                actionName(name)
        {
            actionServer.start();
            ROS_INFO("Simple action server started");
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fibonacci");
    FibonacciActionServer fibonacci("fibonacci");

    ros::spin();
    return 0;
}