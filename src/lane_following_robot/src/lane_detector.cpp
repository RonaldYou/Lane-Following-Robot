#include "lane_following_robot/lane_detector.h"

LaneDetector::LaneDetector() : Node("lane_detector"), hue_low_(20), hue_high_(30), sat_low_(100), sat_high_(255), val_low_(100), val_high_(255) {
    image_transport::ImageTransport it(this);
    image_sub_ = it.subscribe("/camera/image_raw", 1, std::bind(&LaneDetector::image_callback, this, std::placeholders::_1));
    lane_center_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/lane_center", 10);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/debug_image", 10);
    RCLCPP_INFO(this->get_logger(), "Lane Detector Node Initialized");
}

void LaneDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat processed_image = process_image(cv_ptr->image);
    geometry_msgs::msg::Point lane_center = detect_lane_center(processed_image);

    lane_center_pub_->publish(lane_center);

    // Publish debug image
    cv_bridge::CvImage debug_msg;
    debug_msg.header = msg->header;
    debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
    debug_msg.image = processed_image;
    debug_image_pub_->publish(*debug_msg.toImageMsg());
}

