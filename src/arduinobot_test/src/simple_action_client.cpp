#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "arduinobot_test/FibonacciAction.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fibonacci_client");
    actionlib::SimpleActionClient<arduinobot_test::FibonacciAction> ac("fibonacci");
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    // create goal class
    ROS_INFO("Action server started, sending a goal");
    arduinobot_test::FibonacciGoal goal;
    goal.goal = 20;

    // Send goal and wait for result
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(30.0));

    auto result = ac.getResult();
    ROS_INFO("Action finished: ");
    
    for (int i : result->sequence) {
        ROS_INFO("%d", i);
    }

    return 0;
}