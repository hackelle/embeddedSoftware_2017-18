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

std::vector <cv::Vec2f> detect_lines_hough(cv::Mat inImage) {
    // constants
    float houghLinesRadius = 5;
    float houghLinesAngles = CV_PI / 180;
    float houghLinesThreshold = 2000;
    int cannyLowThreshold = 5;
    int cannyRatio = 3; // recommended 3
    int cannyKernalSize = 3; // recommended 3

    cv::Mat smallMat, greyMat, blurMat, cannyMat, edgeMat, perspectiveMat, fourierMat, lineMat;

    // Transpose and flip to get portrait mode
    cv::transpose(inImage, inImage);
    cv::flip(inImage, inImage, 1);

    // Scale down image to 270*480 pixel (1/16 of 1080*1920) and reduce color depth
    cv::Size size(270, 480);
    cv::resize(inImage, smallMat, size);
    colorReduce(smallMat);


    // Make grey scale
    cv::cvtColor(smallMat, greyMat, CV_BGR2GRAY);

    // Blur image
    cv::blur(greyMat, blurMat, cv::Size(3, 3));

    // Detect edges with Canny
    cv::Canny(blurMat, cannyMat, cannyLowThreshold, cannyLowThreshold * cannyRatio, cannyKernalSize);

    // Make picture of only white edges on black background
    edgeMat = cv::Scalar::all(0);
    smallMat.copyTo(edgeMat, cannyMat);


    // fourier transform
    //fourierMat = fourier_transform(greyMat);

    // detect lines
    std::vector <cv::Vec2f> lines;
    /*cv::HoughLines(greyMat, lines, houghLinesRadius, houghLinesAngles,
                   houghLinesThreshold, 0, 0 );
    std::cout << lines.size() << std::endl;

    lineMat = greyMat.clone();
    // Draw lines in image
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(lineMat, pt1, pt2, cv::Scalar(0, 0, 255, 255), 3, CV_AA);
    }
     */

    // Display images
    cv::imshow("Robot perspective", inImage);
    //cv::imshow("Scaled image", smallMat);
    cv::imshow("Grey image", greyMat);
    //cv::imshow("Blur image", blurMat);
    //cv::imshow("Canny image", cannyMat);
    cv::imshow("Edge image", edgeMat);
    /*if (!fourierMat.empty())
        cv::imshow("Fourier image", fourierMat);
    cv::imshow("Line detection", lineMat);
*/
    return lines;
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

    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input
    srcQuad[0] = Point2f(-30, -60);
    srcQuad[1] = Point2f(src.cols + 50, -50);
    srcQuad[2] = Point2f(src.cols + 100, src.rows + 50);
    srcQuad[3] = Point2f(-50, src.rows + 50);

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