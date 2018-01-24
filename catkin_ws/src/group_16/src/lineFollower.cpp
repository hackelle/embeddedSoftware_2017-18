#include "lineFollower.h"

//#define DEBUG_SAVE
#define DEBUG_LOAD

// show the picture
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        // use the bridge to convert msg image to a matrix
        cv::Mat inImage = cv_bridge::toCvShare(msg, "bgr8")->image;

        // save if debugging-saving
#ifdef DEBUG_SAVE
        imwrite(image_path, inImage);
#endif // DEBUG_SAVE


        std::cout << "detecting line" << std::endl;
        float angular_vel_z = detect_line_simple(inImage);
        std::cout << "done" << std::endl;

        // always go full speed (no robot can go 1m/s)
        sendMessage(0.11,0,0,
                    0,0,angular_vel_z);

        // wait a short time for image-display (25 for mobile, 100 for debug)
        cv::waitKey(500);
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

#ifdef DEBUG_LOAD
    ros::Publisher debug_pub = nh.advertise<sensor_msgs::CompressedImage>("camera/image/compressed", 1);

    // create message to publish each time
    cv::Mat img = cv::imread(image_path, 1); // << image MUST be contained here
    cv_bridge::CvImage img_bridge;
    sensor_msgs::CompressedImage img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.seq = 1234; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toCompressedImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
#endif

    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // subscribe to topic
    // start image processing
    cv::namedWindow("Robot perspective", CV_WINDOW_NORMAL);
    //cv::namedWindow("Scaled image", CV_WINDOW_NORMAL);
    cv::namedWindow("Grey image", CV_WINDOW_NORMAL);
    //cv::namedWindow("Blur image", CV_WINDOW_NORMAL);
    //cv::namedWindow("Canny image", CV_WINDOW_NORMAL);
    //cv::namedWindow("Edge image", CV_WINDOW_NORMAL);
    cv::namedWindow("Perspective image", CV_WINDOW_NORMAL);
    cv::namedWindow("Debug image", CV_WINDOW_NORMAL);
    cv::namedWindow("Line image", CV_WINDOW_NORMAL);
    // FIXME: for some reason the last window is not shown
    //cv::namedWindow("dummy", CV_WINDOW_NORMAL);

    lastTime = ros::Time::now();
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("camera/image", 1, imageCallback,
                       ros::VoidPtr(), image_transport::TransportHints("compressed"));
    std::cout << "Linefollower started" << std::endl;

    while (!g_request_shutdown) {

        ros::spinOnce();

#ifdef DEBUG_LOAD
        // send the saved&reloaded image to trigger the callback
        std::cout << "Fake sending image" << std::endl << std::endl << std::endl;
        debug_pub.publish(img_msg);
#endif // DEBUG_LOAD

    }

    /*
     * Overwrite the shut down routine, as window has to be closed but
     * throws a segfault if not destroyed.
     */
    cv::destroyWindow("Robot perspective");
    //cv::destroyWindow("Scaled image");
    cv::destroyWindow("Grey image");
    //cv::destroyWindow("Blur image");
    //cv::destroyWindow("Canny image");
    //cv::destroyWindow("Edge image");
    cv::destroyWindow("Perspective image");
    cv::destroyWindow("Debug image");
    cv::destroyWindow("Line image");
    ros::shutdown();

    return 0;
}

// overwrite for shutdown
void SigIntHandler(int sig) {
    g_request_shutdown = 1;
}

void sendMessage(float linear_x, float linear_y, float linear_z,
                 float angular_x, float angular_y, float angular_z){

    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = linear_x;
    twist_msg.linear.y = linear_y;
    twist_msg.linear.z = linear_z;

    twist_msg.angular.x = angular_x;
    twist_msg.angular.y = angular_y;
    twist_msg.angular.z = angular_z;

    ROS_INFO("\nSetting: lin       ang"
             "\n      x: %3f  %3f"
             "\n      y: %3f  %3f"
             "\n      z: %3f  %3f",
             twist_msg.linear.x, twist_msg.angular.x,
             twist_msg.linear.y, twist_msg.angular.y,
             twist_msg.linear.z, twist_msg.angular.z);

    twist_pub.publish(twist_msg);
}