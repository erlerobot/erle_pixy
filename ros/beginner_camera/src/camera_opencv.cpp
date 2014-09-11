#include "ros/ros.h"

#include <sensor_msgs/fill_image.h>
//#include <self_test/self_test.h>
#include <image_transport/image_transport.h>

#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image");

    ros::NodeHandle n;

    image_transport::Publisher image_pub;

    image_transport::ImageTransport it(n);
    image_pub = it.advertise("image_raw", 1);

    sensor_msgs::Image img_;

    ros::Rate loop_rate(25);

    cv::Mat frame;
    cv::VideoCapture cap(0);

    while (ros::ok()){
        cap>> frame;

        fillImage(img_, "bgr8", frame.rows, frame.cols, frame.channels() * frame.cols, frame.data);

        image_pub.publish(img_);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
