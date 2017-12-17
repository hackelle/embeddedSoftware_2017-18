#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // include own message type

void twistCallback(const geometry_msgs::Twist &twist_msg) {
    ROS_INFO("\nReceived Message:\n    x:%f     y: %f\n-------------------------------", twist_msg.linear.x,
             twist_msg.linear.y);
}

int main(int argc, char **argv) {
    // init with dummy name
    ros::init(argc, argv, "twist_msg_dummy");

    // create handle
    ros::NodeHandle n;

    // subscribe to topic
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, twistCallback);

    ROS_INFO("\nDummy active!");

    // enter loop
    ros::spin();

    return 0;
}
