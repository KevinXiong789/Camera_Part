/*
#include "rclcpp/rclcpp.hpp"
#include <limits>
#include <cmath>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"

#include <memory>
#include <iostream>
#include <vector>

#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
//#include "seher/move_robot/move_robot.h"
//#include <seher_msgs/msg/handover.hpp>
*/

/* 
// *******************source code*******************************can run
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class PositionSubscriher : public rclcpp::Node {
public:
  PositionSubscriher() : Node("right_hand_position_subscriber") {
    // Subscribe to the "right_hand_position" topic with a callback function
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/Nuitrack/right_hand_position", 10, std::bind(&PositionSubscriher::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Receive the x, y, and d coordinates from the Float32MultiArray message
    std::vector<float> data = msg->data;
    float right_x = data[0];
    float right_y = data[1];
    float right_d = data[2];
    // Process the received coordinates as needed
    RCLCPP_INFO(this->get_logger(), "Received right hand x: %.4f, y: %.4f, d: %.4f", right_x, right_y, right_d);
    if (right_x >= -0.1 && right_x <= 0.1 &&
        right_y >= -0.1 && right_y <= 0.1 &&
        right_d >= 0.4 && right_d <= 0.6) {
      // Print prompt if right hand is within the fixed 3D space
      RCLCPP_INFO(this->get_logger(), "Right hand is within the fixed 3D space");
    } 

  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionSubscriher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/


// *******************Nuitrack Handover triggered, tool in hand not yet***********************************can run
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

class PositionSubscriber : public rclcpp::Node {
public:
  PositionSubscriber() : Node("right_hand_position_subscriber") {
    // Subscribe to the "right_hand_position" topic with a callback function
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/Nuitrack/right_hand_position", 1, std::bind(&PositionSubscriber::callback, this, std::placeholders::_1));
    prev_x_ = 0.0;
    prev_y_ = 0.0;
    prev_d_ = 0.0;
    prev_time_ = this->now();
    //timer_duration_ = 0;

    // Create publisher for bool topic
    bool_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/handover_trigger", 10);
  }

private:
  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Receive the x, y, and d coordinates from the Float32MultiArray message
    std::vector<float> sub_data = msg->data;
    float right_x = sub_data[0];
    float right_y = sub_data[1];
    float right_d = sub_data[2];

    auto handover_msg = std_msgs::msg::Bool();
    
    // Process the received coordinates as needed
    //RCLCPP_INFO(this->get_logger(), "Received right hand x: %.4f, y: %.4f, d: %.4f", right_x, right_y, right_d);
    if (right_x >= -0.1 && right_x <= 0.1 &&
        right_y >= -0.1 && right_y <= 0.1 &&
        right_d >= 0.4 && right_d <= 0.6) {
      // Check if right hand is within the fixed 3D space
      RCLCPP_INFO(this->get_logger(), "Right hand is in workcell");
    
      // Calculate the Euclidean distance between current and previous positions
      float distance = std::sqrt(std::pow(right_x - prev_x_, 2) +
                                 std::pow(right_y - prev_y_, 2) +
                                 std::pow(right_d - prev_d_, 2));
      if (distance > 0.01) {
        // If hand position remains approximately unchanged, reset the timer
        timer_duration_ = 0;
      } else {
        // If hand position changes, increment the timer duration
        timer_duration_ += (this->now() - prev_time_).seconds();
      }

      if (timer_duration_ > static_hand_duration_) {
        // If timer duration exceeds static hand duration, trigger handover
        RCLCPP_INFO(this->get_logger(), "Handover triggered!!!!!!!!!!!!!!!!!!");
        
        handover_msg.data = true;
        bool_publisher_->publish(handover_msg);

      } else {
        RCLCPP_INFO(this->get_logger(), "Handover not triggered");
      }
      
      // Update previous position and time
      prev_x_ = right_x;
      prev_y_ = right_y;
      prev_d_ = right_d;
      prev_time_ = this->now();
      handover_msg.data = false;

    } else {
      RCLCPP_INFO(this->get_logger(), "Right hand is not in workcell");
      // Update previous position and time
      prev_x_ = right_x;
      prev_y_ = right_y;
      prev_d_ = right_d;
      prev_time_ = this->now();
      handover_msg.data = false;

    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_publisher_;
  float prev_x_;
  float prev_y_;
  float prev_d_;
  rclcpp::Time prev_time_;
  float timer_duration_;
  float static_hand_duration_ = 3.0; // Static hand duration in seconds
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}




