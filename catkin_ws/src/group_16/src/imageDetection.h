//
// Created by hackelle on 22.12.17.
//

#ifndef CATKIN_WS_IMAGEDETECTION_H
#define CATKIN_WS_IMAGEDETECTION_H

// standard
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <list>
#include <cmath>
#include <stdio.h>
#include <iostream>
// open cv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"


/**
 * Performs fourier transformation on a cv::Mat object.
 * @param src cv::Mat matrix of a picture
 * @return show-able fourier transformed representation of src or src, if src is empty
 */
cv::Mat fourier_transform(cv::Mat src);

/**
 * Rotates, reduces size and makes image black and white
 * @param inImage Image to work on
 * @param threshold [optional] threshold for black value, standard = 128
 * @return a black and white image, 90° rotated, 270*480
 */
cv::Mat prepare_image(cv::Mat inImage, int threshold = 128);

/**
 * Reduces the color depth of an image
 * @param image the in and output cv::Mat. Runs inplace for performance
 * @param div [optional] the new color depth, standard = 128 (means 2 colors per channel)
 */
void colorReduce(cv::Mat& image, int div = 128);

/**
 * Use hough transform and linear approximation to find a line
 *
 * @param inImage cv::Mat matrix of picture to find line curvature in
 * @return float of best angular velocity fitting in between the line edges
 */
float detect_line(cv::Mat inImage);

/**
 * Use hough transform to find a line
 *
 * @param inImage cv::Mat matrix of picture to find line curvature in
 * @return float of best angular velocity fitting in between the line edges
 */
float detect_line_hough(cv::Mat inImage);

/**
 * Performs a line detection and displays some steps:
 * 1) Rotate 90 deg clockwise (Shown in cv::namedWindow "Robot perspective").
 * 2) Scale image to 270*480 (Shown in cv::namedWindow "Scaled image").
 * 3) Make image greyscale (Shown in cv::namedWindow "Grey image") and reduce color depth.
 * 4) Canny detect edges in the picture.
 * 5) Take average of x_coord to approximate linear.
 *
 * @param inImage cv::Mat matrix of picture to find line curvature in
 * @return float of best angular velocity fitting in between the line edges
 */
float detect_line_linear(cv::Mat inImage);
float detect_line_simple(cv::Mat inImage);

/**
 * Creates and returns a random alpha numeric string
 * @param length length of the string
 * @return random alpha numeric string
 */
std::string random_string(size_t length);

/**
 * Performs a perspective transformation optimized for usage on the arduino Robot
 * in combination with a ZTE AXON 7 camera in the module "Embedded Software" in
 * Q2, TU DELFT 2017/18.
 * @param src The source image
 * @param dst The destination image
 */
void perspectiveTransformForRobot(cv::Mat &src, cv::Mat &dst);

/**
 * Fits different curvatures into a point array and returns speed and angular velocity for best fitting circle
 * @param points point-vector to work on
 * @param speed return value, best fitting speed
 * @param angle return value, best fitting angular velocity
 * @param offset amount of pixel "below" picture until real 0 of the robot
 */
void fit_lines(std::vector<cv::Point> points, double *speed, double *angle, int offset);

#endif //CATKIN_WS_IMAGEDETECTION_H
