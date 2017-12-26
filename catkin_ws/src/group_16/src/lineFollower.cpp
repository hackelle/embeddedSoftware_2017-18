#include "lineFollower.h"

// show the picture
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        // Get the msg image, convert to a matrix
        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg, "bgr8")->image;

        detect_lines_hough(InImage);

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

    // FIXME: Something with killing doesn't shut down correctly
    //signal(SIGINT, SigIntHandler);

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // subscribe to topic
    // start image processing
    cv::namedWindow("Robot perspective", CV_WINDOW_NORMAL);
    cv::namedWindow("Scaled image", CV_WINDOW_NORMAL);
    cv::namedWindow("Grey image", CV_WINDOW_NORMAL);
    cv::namedWindow("Blur image", CV_WINDOW_NORMAL);
    cv::namedWindow("Canny image", CV_WINDOW_NORMAL);
    cv::namedWindow("Edge image", CV_WINDOW_NORMAL);
    //cv::namedWindow("Fourier image", CV_WINDOW_NORMAL);
    //cv::namedWindow("Line detection", CV_WINDOW_NORMAL);
    // FIXME: for some reason the last window is not shown
    //cv::namedWindow("dummy", CV_WINDOW_NORMAL);

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
    cv::destroyWindow("Blur image");
    cv::destroyWindow("Canny image");
    cv::destroyWindow("Edge image");
    //cv::destroyWindow("Fourier image");
    //cv::destroyWindow("Line detection");
    ros::shutdown();

    return 0;
}

// overwrite for shutdown
void SigIntHandler(int sig) {
    g_request_shutdown = 1;
}