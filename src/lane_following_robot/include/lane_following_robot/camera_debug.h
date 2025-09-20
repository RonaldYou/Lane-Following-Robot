#ifndef CAMERA_DEBUG_HPP_
#define CAMERA_DEBUG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class CameraDebug : public rclcpp::Node{
    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        image_transport::Subscriber image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

        int image_count_;
    public: 
        CameraDebug();
}