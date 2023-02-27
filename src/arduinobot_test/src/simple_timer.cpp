#include <ros/ros.h>

void timerCallback(const ros::TimerEvent &event) {
    ROS_INFO("Called timer callback function");
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "timer_node");
    ros::NodeHandle nodeHandle;
    ros::Duration timerDuration(1);
    ros::Timer timer = nodeHandle.createTimer(timerDuration, timerCallback);
    ros::spin();

    return 0;
}