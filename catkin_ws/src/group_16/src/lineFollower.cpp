#include "lineFollower.h"

// show the picture
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        // Get the msg image, convert to a matrix
        cv::Mat InImage, smallMat, greyMat, fourierMat, lineMat;
        InImage = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Transpose and flip to get portrait mode
        cv::transpose(InImage, InImage);
        cv::flip(InImage, InImage, 1);

        // Scale down image to 270*480 pixel (1/16 of 1080*1920)
        cv::Size size(270,480);
        cv::resize(InImage, smallMat, size);

        // Make grey scale
        cv::cvtColor(smallMat, greyMat, CV_BGR2GRAY);

        // fourier transform
        fourierMat = fourier_transform(greyMat);

        // detect lines
        std::vector<cv::Vec2f> lines;
        cv::HoughLines(greyMat, lines, 5, CV_PI/180, 2000, 0, 0 );
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

        // Display images
        cv::imshow("Robot perspective", InImage);
        cv::imshow("Scaled image", smallMat);
        cv::imshow("Grey image", greyMat);
        if (!fourierMat.empty())
            cv::imshow("Fourier image", fourierMat);
        cv::imshow("Line detection", lineMat);
        cv::waitKey(100);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'jpg'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv) {
    // overwrite shutdown
    ros::init(argc, argv, "line_follower", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;

    // TODO: Something with killing doesn't shut down correctly
    //signal(SIGINT, SigIntHandler);

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // subscribe to topic
    // start image processing
    cv::namedWindow("Robot perspective", CV_WINDOW_NORMAL);
    cv::namedWindow("Scaled image", CV_WINDOW_NORMAL);
    cv::namedWindow("Grey image", CV_WINDOW_NORMAL);
    cv::namedWindow("Fourier image", CV_WINDOW_NORMAL);
    cv::namedWindow("Line detection", CV_WINDOW_NORMAL);
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
    cv::destroyWindow("Robot perspective");
    cv::destroyWindow("Scaled image");
    cv::destroyWindow("Grey image");
    cv::destroyWindow("Fourier image");
    cv::destroyWindow("Line detection");
    ros::shutdown();

    return 0;
}

cv::Mat fourier_transform(cv::Mat src){
    using namespace cv;
    if( src.empty())
        return src;

    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDFTSize( src.rows );
    int n = getOptimalDFTSize( src.cols ); // on the border add zero values
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
    int cx = magI.cols/2;
    int cy = magI.rows/2;

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

// overwrite for shutdown
void SigIntHandler(int sig) {
    g_request_shutdown = 1;
}