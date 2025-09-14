#include "lane_following_robot/lane_detector.h"

LaneDetector::LaneDetector() : Node("lane_detector"), hue_low_(20), hue_high_(30), sat_low_(100), sat_high_(255), val_low_(100), val_high_(255) {
    image_transport::ImageTransport it(this);
    image_sub_ = it.subscribe("/camera/image_raw", 1, std::bind(&LaneDetector::image_callback, this, std::placeholders::_1));
    lane_center_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/lane_center", 10);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/debug_image", 10);
    RCLCPP_INFO(this->get_logger(), "Lane Detector Node Initialized");
}

