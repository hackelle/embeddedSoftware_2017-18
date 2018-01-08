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
 * Performs a line detection and displays steps:
 * 1) Rotate 90 deg clockwise (Shown in cv::namedWindow "Robot perspective").
 * 2) Scale image to 270*480 (Shown in cv::namedWindow "Scaled image").
 * 3) Make image greyscale (Shown in cv::namedWindow "Grey image").
 * 4) Fourier transform image (Shown in cv::namedWindow "Fourier image").
 * 5) Hough transform image to detect lines (Shown in cv::namedWindow "Line detection").
 *
 * @param inImage cv::Mat matrix of picture to find lines on
 * @return std::vector<cv::Vec2f> vector of lines
 */
std::vector<cv::Vec2f> detect_lines_hough(cv::Mat inImage);

/**
 * Reduces the color depth of an image
 * @param image the in and output cv::Mat. Runs inplace for performance
 * @param div [optional] the new color depth, standard = 64
 */
void colorReduce(cv::Mat& image, int div);
void colorReduce(cv::Mat& image);

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

#endif //CATKIN_WS_IMAGEDETECTION_H
