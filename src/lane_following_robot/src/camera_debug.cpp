#include "lane_following_robot/camera_debug.h"

CameraDebug::CameraDebug(): Node("camera_debug"), image_count_(0){
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&CameraDebug::image_callback, this, std::placeholders::_1));
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/debug_image", 10);
    RCLCPP_INFO(this->get_logger(), "Camera Debug Node Initialized");
}

void CameraDebug::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_count_++;

        cv::Mat debug_image = cv_ptr->image.clone();

        std::string frame_text = "Frame: " + std::to_string(image_count_);
        cv::putText(debug_image, frame_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

        sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
        debug_image_pub_->publish(*debug_msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraDebug>();
    
    RCLCPP_INFO(node->get_logger(), "Starting camera debug node...");
    
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Camera debug node shutting down.");
    rclcpp::shutdown();
    return 0;
}