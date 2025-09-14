#ifndef LANE_DETECTOR_HPP_
#define LANE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class LaneDetector : public rclcpp::Node{
    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        cv::Mat process_image(const cv::Mat& image);
        geometry_msgs::msg::Point detect_lane_center(const cv::Mat& processed_image);
        
        // ROS2 interfaces
        image_transport::Subscriber image_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr lane_center_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
        
        // Image processing parameters
        int hue_low_, hue_high_;
        int sat_low_, sat_high_;
        int val_low_, val_high_;

    public:
        LaneDetector();
};

#endif // LANE_DETECTOR_HPP_