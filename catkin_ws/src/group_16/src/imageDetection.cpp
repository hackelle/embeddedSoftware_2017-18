//
// Created by hackelle on 22.12.17.
//

#include "imageDetection.h"

cv::Mat fourier_transform(cv::Mat src) {
    using namespace cv;
    if (src.empty())
        return src;

    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDFTSize(src.rows);
    int n = getOptimalDFTSize(src.cols); // on the border add zero values
    copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    dft(complexI, complexI);            // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];

    magI += Scalar::all(1);                    // switch to logarithmic scale
    log(magI, magI);

    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols / 2;
    int cy = magI.rows / 2;

    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
    // viewable image form (float between values 0 and 1).

    return magI; // return the result
}

float detect_line(cv::Mat inImage){
    return 0.3 * detect_line_hough(inImage) + 0.7 * detect_line_linear(inImage);
}

float detect_line_hough(cv::Mat inImage){
    // constants
    int cannyLowThreshold = 5;
    int cannyRatio = 3; // recommended 3
    int cannyKernalSize = 3; // recommended 3

    cv::Mat smallMat, greyMat, blurMat, cannyMat, edgeMat,
            perspectiveMat, fourierMat, lineMat;

    // Transpose and flip to get portrait mode
    cv::transpose(inImage, inImage);
    cv::flip(inImage, inImage, 1);

    // Scale down image to 270*480 pixel (1/16 of 1080*1920) and reduce color depth
    cv::Size size(270, 480);
    cv::resize(inImage, smallMat, size);
    colorReduce(smallMat, 8);


    // Make grey scale
    cv::cvtColor(smallMat, greyMat, CV_BGR2GRAY);

    // Blur image
    cv::blur(greyMat, blurMat, cv::Size(3, 3));

    // Detect edges with Canny
    cv::Canny(blurMat, cannyMat, cannyLowThreshold, cannyLowThreshold * cannyRatio, cannyKernalSize);

    // Make picture of only colorful edges on black background
    edgeMat = cv::Scalar::all(0);
    smallMat.copyTo(edgeMat, cannyMat);

    // transform with perspective (canny and color for lines later)
    perspectiveTransformForRobot(cannyMat, perspectiveMat);
    smallMat.copyTo(lineMat);
    perspectiveTransformForRobot(lineMat, lineMat);

    // crop the image, discard first x cols pixel
    // FIXME: Why does this work?
    cv::Mat croppedMat = perspectiveMat.clone();
    for (int j = 0; j < croppedMat.rows; ++j) {
        for (int i = 0; i < croppedMat.cols; ++i) {
            croppedMat.at<cv::Vec3b>(i,j)[0] = 0;
            croppedMat.at<cv::Vec3b>(i,j)[1] = 0;
            croppedMat.at<cv::Vec3b>(i,j)[2] = 0;
        }
    }

    // detect lines in hough and draw lines in picture
    std::vector<cv::Vec4i> lines;
    int overlap = 30;
    cv::HoughLinesP(croppedMat, lines, 1, CV_PI/180, overlap, 100, 40);
    // draw lines, get average angle
    double angle = 0;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line( lineMat, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
        angle += atan2(l[1] - l[3], l[0] - l[2]);
    }
    angle /= lines.size();

    // Display images
    cv::imshow("Robot perspective", inImage);
    //cv::imshow("Scaled image", smallMat);
    cv::imshow("Grey image", greyMat);
    //cv::imshow("Blur image", blurMat);
    cv::imshow("Canny image", cannyMat);
    //cv::imshow("Edge image", edgeMat);
    cv::imshow("Perspective image", perspectiveMat);
    cv::imshow("Line image", lineMat);
    return angle;
}

float detect_line_linear(cv::Mat inImage) {
    // constants
    int cannyLowThreshold = 5;
    int cannyRatio = 3; // recommended 3
    int cannyKernalSize = 3; // recommended 3

    cv::Mat smallMat, greyMat, blurMat, cannyMat, edgeMat,
            perspectiveMat, fourierMat, lineMat;

    // Transpose and flip to get portrait mode
    cv::transpose(inImage, inImage);
    cv::flip(inImage, inImage, 1);

    // Scale down image to 270*480 pixel (1/16 of 1080*1920) and reduce color depth
    cv::Size size(270, 480);
    cv::resize(inImage, smallMat, size);
    colorReduce(smallMat, 8);


    // Make grey scale
    cv::cvtColor(smallMat, greyMat, CV_BGR2GRAY);

    // Blur image
    cv::blur(greyMat, blurMat, cv::Size(3, 3));

    // Detect edges with Canny
    cv::Canny(blurMat, cannyMat, cannyLowThreshold, cannyLowThreshold * cannyRatio, cannyKernalSize);

    // Make picture of only colorful edges on black background
    // (copy everything from small in edge, where canny is not black)
    edgeMat = cv::Scalar::all(0);
    smallMat.copyTo(edgeMat, cannyMat);

    // transform with perspective
    perspectiveTransformForRobot(cannyMat, perspectiveMat);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // Find contours
    cv::findContours(perspectiveMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // how much of the bottom part of the picture is relevant for our calculation?
    double relevant_part = 0.2;

    // calculate approx x
    double av_x = 0;
    int points = 0;
    for (auto & c : contours){
        for (auto & p: c){
            if (p.y < perspectiveMat.cols * (1-relevant_part)){
                av_x += p.x;
                points++;
            }
        }
    }
    av_x /= points;
    cv::line(perspectiveMat, cv::Point (perspectiveMat.cols/2 ,perspectiveMat.rows-1),
                cv::Point(av_x, perspectiveMat.rows-100), cv::Scalar(255, 0, 0), 3);

    // calculate the angle of rotation based on the distance
    double distance_to_middle = av_x-perspectiveMat.cols/2;
    double angle = distance_to_middle *  3.14159265/ perspectiveMat.cols;

    // Display images
    cv::imshow("Robot perspective", inImage);
    //cv::imshow("Scaled image", smallMat);
    cv::imshow("Grey image", greyMat);
    //cv::imshow("Blur image", blurMat);
    cv::imshow("Canny image", cannyMat);
    //cv::imshow("Edge image", edgeMat);
    cv::imshow("Perspective image", perspectiveMat);

    return angle;
}

void colorReduce(cv::Mat &image) {
    colorReduce(image, 64);
}

void colorReduce(cv::Mat &image, int div) {
    int nl = image.rows;                    // number of lines
    int nc = image.cols * image.channels(); // number of elements per line

    for (int j = 0; j < nl; j++) {
        // get the address of row j
        uchar *data = image.ptr<uchar>(j);

        for (int i = 0; i < nc; i++) {
            // process each pixel
            data[i] = data[i] / div * div + div / 2;
        }
    }
}

std::string random_string(size_t length) {
    auto randChar = []() -> char {
        const char charset[] =
                "0123456789"
                        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[rand() % max_index];
    };
    std::string str(length, 0);
    std::generate_n(str.begin(), length, randChar);
    return str;
}

void perspectiveTransformForRobot(cv::Mat &src, cv::Mat &dst) {
    using namespace cv;
    // Input Quadilateral or Image plane coordinates
    Point2f srcQuad[4];

    // Output Quadilateral or World plane coordinates
    Point2f dstQuad[4];

    // Lambda Matrix
    Mat lambda(2, 4, CV_32FC1);

    // Set the lambda matrix the same type and size as input
    lambda = Mat::zeros(src.rows, src.cols, src.type());

    // "Destination points"
    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input
    int col_offset = 350;
    srcQuad[0] = Point2f(0, 0);
    srcQuad[1] = Point2f(src.cols -1, 0);
    srcQuad[2] = Point2f(src.cols + col_offset, src.rows-1);
    srcQuad[3] = Point2f(-col_offset, src.rows-1);
    /*
    srcQuad[0] = Point2f(-30, -60);
    srcQuad[1] = Point2f(src.cols + 50, -50);
    srcQuad[2] = Point2f(src.cols + 100, src.rows + 50);
    srcQuad[3] = Point2f(-50, src.rows + 50);*/

    // "Source points"
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    dstQuad[0] = Point2f(0, 0);
    dstQuad[1] = Point2f(src.cols - 1, 0);
    dstQuad[2] = Point2f(src.cols - 1, src.rows - 1);
    dstQuad[3] = Point2f(0, src.rows - 1);

    // Get the Perspective Transform Matrix i.e. lambda
    lambda = getPerspectiveTransform(srcQuad, dstQuad);

    // Apply the Perspective Transform just found to the src image
    warpPerspective(src, dst, lambda, dst.size());
}

void fit_lines(cv::Mat &perspectiveMat, double &smallest_distance_speed, double &smallest_distance_ang,
               double &smallest_distance_radius, double* distances, int y_pixel_offset){

    double smallest_distance = HUGE_VAL;
    double PI = 3.14159265358;
    int counter = 0;

    for (double i = 0.07; i < 0.18; i+=0.02) {
        // try for linear speeds 7,9,11,13,15,17 [cm]
        for (double j = -4.0/8*PI; j < 5.0/8*PI; j+=1.0/8*PI){
            // try for angular speeds -4/8 pi .. 4/8 pi in 1/8 pi steps (-90° to +90°)
            double angular_z = 0.0001;
            if (j != 0) angular_z = j;
            double linear_x = i;

            // calculate radius of the circle
            double radius = linear_x * (2*PI/angular_z) * y_pixel_offset;

            // calculate distance of each point to this circle
            std::vector<cv::Point> locations;   // output, locations of non-zero (=white) pixels
            cv::findNonZero(perspectiveMat, locations);
            // access pixel coordinates
            // add all distances (weighted by distance to lower end of picture)
            distances[counter] = 0;
            for (auto & loc :locations){
                distances[counter] += abs(sqrt(pow(loc.x,2) + pow(loc.y,2)) - radius)
                                      * ((float)loc.y/perspectiveMat.cols);
            }

            if (distances[counter] < smallest_distance){
                smallest_distance_speed = linear_x;
                smallest_distance_ang = angular_z;
                smallest_distance_radius = radius;
                smallest_distance = distances[counter];
                std::cout << "New smallest distance = " <<  smallest_distance << " @ (" <<
                          smallest_distance_speed << "|" << smallest_distance_ang << ")" << std::endl;
            }
            counter++;
        }
    }
}