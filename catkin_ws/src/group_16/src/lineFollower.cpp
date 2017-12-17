#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // include twist message type
//#include "sensor_msgs/CompressedImage.h" // include compressed image type

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <list>
#include <cmath>
#include <stdio.h>
#include <ros/package.h>


// Subscriber to bottom camera
image_transport::Subscriber sub;

// Time control
ros::Time lastTime;

//For SIGINT
sig_atomic_t volatile g_request_shutdown = 0;


// show the picture
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Get the msg image
        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg,"bgr8" )->image;
        cv::imshow("view", InImage);
        cv::waitKey(100);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'jpg'.", msg->encoding.c_str());
    }
}

// overwrite for shutdown
void SigIntHandler(int sig) {
    g_request_shutdown = 1;
}


int main(int argc, char **argv) {
    // overwrite shutdown
    ros::init(argc, argv, "line_follower", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // subscribe to topic
    // start image processing
    cv::namedWindow("Robot perspective",CV_WINDOW_NORMAL);
    lastTime = ros::Time::now();
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("camera/image", 1, imageCallback,
                       ros::VoidPtr(), image_transport::TransportHints("compressed"));

    // more then 10 hz is not possible with the camera
    // at least have a somehow constant send rate
    ros::Rate loop_rate(10);

    int count = 0;
    while (!g_request_shutdown) {
        geometry_msgs::Twist twist_msg;

        twist_msg.linear.x = count;
        twist_msg.linear.y = count - 1;

        ROS_INFO("\nSetting:\n    x:%f     y: %f", twist_msg.linear.x, twist_msg.linear.y);

        twist_pub.publish(twist_msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    /*
     * Overwrite the shut down routine, as window has to be closed but
     * throws a segfault if not destroyed.
     */
    cv::destroyWindow("view");
    ros::shutdown();

    return 0;
}
