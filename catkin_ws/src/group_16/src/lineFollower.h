//
// Created by hackelle on 20.12.17.
//

#ifndef CATKIN_WS_LINEFOLLOWER_H
#define CATKIN_WS_LINEFOLLOWER_H

// standard
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <list>
#include <cmath>
#include <stdio.h>
// ros
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // include twist message type
#include <image_transport/image_transport.h>
#include <ros/package.h>
// open cv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
// own package
#include "imageDetection.h"


// Subscriber to bottom camera
image_transport::Subscriber sub;

// Time control
ros::Time lastTime;

//For SIGINT
sig_atomic_t volatile g_request_shutdown = 0;


void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void SigIntHandler(int sig);
int main(int argc, char **argv);

#endif //CATKIN_WS_LINEFOLLOWER_H
