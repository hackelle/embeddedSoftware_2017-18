//
// Created by hackelle on 22.12.17.
//

#include "imageDetection.h"
class Point{
public:
    double x;
    double y;

    Point(){
        this->x = 0;
        this->y = 0;
    };

    Point(double x, double y){
        this->x = x;
        this->y = y;
    }

    double distance(Point p){
        return sqrt(pow(this->x-p.x, 2) + pow(this->y-p.y, 2));
    }

    std::string toString(){
        std::stringstream ss;
        ss << "(" << this->x << ", " << this->y << ") ";
        return ss.str();
    }
};

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

cv::Mat prepare_image(cv::Mat inImage, int threshold /*= 128*/){
    cv::Mat smallMat, greyMat;

    cv::transpose(inImage, inImage);
    cv::flip(inImage, inImage, 1);

    cv::Size size(270, 480);

    cv::resize(inImage, smallMat, size);
    colorReduce(smallMat);

    // Make grey scale
    cv::cvtColor(smallMat, greyMat, CV_BGR2GRAY);
    greyMat = greyMat > threshold;

    return greyMat;
}

float detect_line(cv::Mat inImage){
    return 0.3 * detect_line_hough(inImage) + 0.3 * detect_line_linear(inImage) + 0.0 * detect_line_simple(inImage);
}

float detect_line_hough(cv::Mat inImage){
    // constants
    int cannyLowThreshold = 5;
    int cannyRatio = 3; // recommended 3
    int cannyKernalSize = 3; // recommended 3

    cv::Mat greyMat, cannyMat, lineMat, perspectiveMat, croppedMat;

    greyMat = prepare_image(inImage);

    // Blur image
    cv::blur(greyMat, cannyMat, cv::Size(3, 3));

    // Detect edges with Canny
    cv::Canny(cannyMat, cannyMat, cannyLowThreshold, cannyLowThreshold * cannyRatio, cannyKernalSize);

    // transform with perspective (canny and color for lines later)
    perspectiveTransformForRobot(cannyMat, perspectiveMat);

    croppedMat = perspectiveMat.clone();

    // startX, startY, width, height
    int startY = croppedMat.rows*3/4;
    cv::Rect myROI(0, startY, croppedMat.cols, croppedMat.rows - startY);

    // Crop the full image to that image contained by the rectangle myROI
    // Note that this doesn't copy the data
    croppedMat = croppedMat(myROI);
    croppedMat.copyTo(lineMat);

    // detect lines in hough and draw lines in picture
    std::vector<cv::Vec4i> lines;
    int overlap = 50;
    cv::HoughLinesP(croppedMat, lines, 1, CV_PI/180, overlap, 50, 40);
    // draw lines, get average angle
    double angle = 0;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line( lineMat, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,255,255), 3, CV_AA);
        angle += atan2(l[1] - l[3], l[0] - l[2]) - M_PI/2;
    }
    if (lines.size()!=0)
        angle /= lines.size();

    angle = fmod(angle,M_PI);

    // Display images
    cv::imshow("Robot perspective", inImage);
    cv::imshow("Grey image", greyMat);

    cv::imshow("Canny image", cannyMat);

    cv::imshow("Perspective image", perspectiveMat);
    cv::imshow("Line image", lineMat);
    return angle;
}

float detect_line_linear(cv::Mat inImage) {
    // constants
    int cannyLowThreshold = 5;
    int cannyRatio = 3; // recommended 3
    int cannyKernalSize = 3; // recommended 3

    cv::Mat greyMat, cannyMat, perspectiveMat;

    greyMat = prepare_image(inImage);

    // Blur image
    cv::blur(greyMat, cannyMat, cv::Size(3, 3));

    // Detect edges with Canny
    cv::Canny(cannyMat, cannyMat, cannyLowThreshold, cannyLowThreshold * cannyRatio, cannyKernalSize);

    // transform with perspective
    perspectiveTransformForRobot(cannyMat, perspectiveMat);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // Find contours
    cv::findContours(perspectiveMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    //cv::findContours(greyMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // how much of the bottom part of the picture is relevant for our calculation?
    double relevant_part_y = 0.05;
    double relevant_part_x_min = 0.15;
    double relevant_part_x_max = 0.85;

    // calculate approx x
    double av_x = 0;
    int points = 0;
    for (auto & c : contours){
        for (auto & p: c){
            if (p.y < perspectiveMat.cols * (1-relevant_part_y) &&
                    p.x > perspectiveMat.cols * relevant_part_x_min &&
                    p.x < perspectiveMat.cols * relevant_part_x_max){
                av_x += p.x;
                points++;
            }
        }
    }
    if (av_x == 0)
        av_x = perspectiveMat.cols/2;
    if (points != 0)
        av_x /= points;

    cv::line(perspectiveMat, cv::Point (perspectiveMat.cols/2 ,perspectiveMat.rows-1),
                cv::Point(av_x, perspectiveMat.rows-100), cv::Scalar(255, 0, 0), 3);

    // calculate the angle of rotation based on the distance
    double distance_to_middle = av_x-perspectiveMat.cols/2;
    double angle = (distance_to_middle < -20 ? -M_PI/3 : distance_to_middle > 20 ? M_PI/3 : 0);

    // Display images
    cv::imshow("Robot perspective", inImage);
    cv::imshow("Grey image", greyMat);

    cv::imshow("Canny image", cannyMat);

    cv::imshow("Perspective image", perspectiveMat);

    return angle;
}

float detect_line_simple(cv::Mat inImage){
    cv::Mat greyMat, perspectiveMat;

    greyMat = prepare_image(inImage);

    // transform with perspective
    perspectiveTransformForRobot(greyMat, perspectiveMat);
    perspectiveMat = greyMat;

    // how much of the bottom part of the picture is relevant for our calculation?
    double relevant_part_y = 0.07;
    double relevant_part_x_min = 0.1;
    double relevant_part_x_max = 0.9;

    cv::Mat debugMat(greyMat.size(), CV_8U, 255);

    // calculate approx x of each col
    std::vector<cv::Point> av_points;

    for (int i = (1-relevant_part_y) * perspectiveMat.rows; i < perspectiveMat.rows; i++){
        double this_av = 0;
        int points = 0;
        for (int j = 0; j < perspectiveMat.cols; j++){
            u_char p = perspectiveMat.at<u_char>(i,j);
            if (p == 0){
                // if its a black pixel, add to average
                this_av += j;
                points++;

                // draw relevent part in debugMat
                debugMat.at<u_char>(cv::Point(j,i - (1-relevant_part_y) * perspectiveMat.rows)) = 180;
            }
        }
        if (points != 0){
            // store average in points
            int av_x = (int)(this_av/points);
            av_points.push_back(cv::Point(av_x, i));
        } else {
            int av_x = greyMat.cols/2;
            av_points.push_back(cv::Point(av_x, i));
        }
    }

    cv::Mat reduced_lines = perspectiveMat.clone();

    double av_x = 0;
    // set only average points to dark
    for(auto &point: av_points){
        reduced_lines.at<u_char>(point) = 170;
        // add offset to draw to top part of matrix
        debugMat.at<u_char>(cv::Point(point.x ,point.y - (1-relevant_part_y) * perspectiveMat.rows)) = 127;
        av_x += point.x;
    }
    av_x /= av_points.size();
    av_x = (av_x == greyMat.cols/2) ? greyMat.cols/2 + 1 : av_x;


    // calculate the angle of rotation based on the distance
    double *angle = new double(0);
    double *speed = new double(151); // 11cm/s; 1px ~ .73mm, 1mm ~ 1.37 px
    int offset = 130; // (in pixel); 212 ~ 15.5cm, 110 ~ 8cm 140 basic ok

    Point robot = Point(135, offset + relevant_part_y * perspectiveMat.rows); // center of robot
    Point goal = Point(av_x, relevant_part_y/2 * perspectiveMat.rows); // goal I want to drive to
    double center_x = (pow(av_x, 2) + pow(robot.y - goal.y, 2) - pow(135,2))/(2*av_x-2*135);
    Point center_circ = Point(center_x, robot.y); // center of driving circle

    //fit_lines(av_points, speed, angle, offset);
    // angle = speed / radius * 400 (correcting for pixel->cm)
    *angle = *speed / (center_circ.x - 135);
    std::cout << "av_x:" << av_x-135 << std::endl;
    std::cout << "Angle:" << *angle << std::endl;

    if (av_x != greyMat.cols/2)
        cv::circle(debugMat, cv::Point(center_circ.x, center_circ.y), center_circ.distance(robot), 191);
    else
        cv::circle(debugMat, cv::Point(center_circ.x, center_circ.y), 1e6, 191);

    // draw center of robot
    debugMat.at<u_char>(cv::Point(robot.x ,robot.y)) = 0;
    debugMat.at<u_char>(cv::Point(goal.x ,goal.y)) = 0;

    // Display images
    cv::imshow("Robot perspective", inImage);
    cv::imshow("Grey image", greyMat);
    cv::imshow("Perspective image", perspectiveMat);
    cv::imshow("Line image", reduced_lines);
    cv::imshow("Debug image", debugMat);

    return *angle;
}

void colorReduce(cv::Mat &image, int div /*= 128*/) {
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

void fit_lines(std::vector<cv::Point> points, double *speed, double *angle, int offset){
    using namespace std;


    int coord_offset = offset; // x pixel "below" seeable pixels

    std::vector<Point> points_cm;
    for(auto &point: points){
        // correct pixel to cm
        // 1 pixel ~ 1mm
        Point p;
        p.y = abs(point.y-479);
        p.y += offset;  // add pixel offset
        p.y *= .001;
        p.x = point.x-135;
        p.x *= .001;

        points_cm.push_back(p);
        cout << p.toString() << endl;
    }

    //points_cm.push_back(Point(0.28, 0.22));
    double smallest_distance = HUGE_VAL;

    for (double i = 0.11; i < 0.13; i+=0.02) {
        // try for linear speeds (7,9,11,13,15,)17 [cm]
        for (double j = -8.0/16*M_PI; j < 9.0/16*M_PI; j+=1.0/16*M_PI){
            // try for angular speeds -4/8 pi .. 4/8 pi in 1/8 pi steps (-90° to +90°)
            double angular_z = 1e-9;
            if (j != 0) angular_z = j;
            double linear_x = i;

            // calculate radius of the circle (center is at (0|r) )
            double radius = linear_x / angular_z;
            double distance_squared = 0;
            for (auto point: points_cm){
                // d = |sqrt((x-0)² + (y-y_0)²) - r|  distance to circle with center (x_0,y_0) and radius r
                // here y_0 = radius and x_0 = 0
                Point center = Point(radius, 0);

                double dist = pow(abs(point.distance(center) - abs(radius)), 1);
                distance_squared += dist;
            }

            cout << "r=" << radius << " d²=" << distance_squared << " (" << linear_x << "|" << angular_z << ")" << endl;
            if (distance_squared <= smallest_distance){
                *speed = linear_x;
                *angle = angular_z;

                smallest_distance = distance_squared;
                std::cout << "New smallest distance = " <<  smallest_distance << std::endl;
                cout << "Center = " << Point(radius,0).toString() << endl << endl;
            }
        }
    }
}