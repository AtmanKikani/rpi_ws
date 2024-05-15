#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

cv::VideoCapture cap1(0);

void launch_cameras() {
    if (!cap1.isOpened()) {
        ROS_ERROR("Error opening camera 0.");
    }
    cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;

    launch_cameras();

    ros::Publisher pub1 = nh.advertise<sensor_msgs::CompressedImage>("/camera_front/image_raw/compressed", 1);

    while (ros::ok()) {
        cv::Mat frame1;
        cap1 >> frame1;
            ROS_INFO("Frame 1 dimensions: %dx%d", frame1.cols, frame1.rows);
            if (frame1.empty()) {
        ROS_ERROR("Empty frame received from camera 0.");
        continue; // Skip this iteration
            }

        sensor_msgs::CompressedImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toCompressedImageMsg();
        pub1.publish(msg1);
    }
    cap1.release();
    return 0;
}

