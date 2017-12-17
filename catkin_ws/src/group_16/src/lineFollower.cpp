#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // include twist message type
//#include "sensor_msgs/CompressedImage.h" // include compressed image type

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

// Subscriber to bottom camera
image_transport::Subscriber sub;

// Time control
ros::Time lastTime;

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

void cameraCallback(const sensor_msgs::CompressedImage &image) {
    ROS_INFO("\nReceived Image: %d\n-------------------------------", image.data[0]);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "line_follower");

    ros::NodeHandle nh;

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // subscribe to topic
    //ros::Subscriber sub = nh.subscribe("/camera/image/compressed", 1, cameraCallback);

    // start image processing
    cv::namedWindow("view",CV_WINDOW_NORMAL);
    lastTime = ros::Time::now();
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("camera/image", 1, imageCallback,
                       ros::VoidPtr(), image_transport::TransportHints("compressed"));

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        geometry_msgs::Twist twist_msg;

        twist_msg.linear.x = count;
        twist_msg.linear.y = count - 1;

        ROS_INFO("\nSetting:\n    x:%f     y: %f", twist_msg.linear.x, twist_msg.linear.y);

        twist_pub.publish(twist_msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    cv::destroyWindow("view");


    return 0;
}
