/*
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
*/



//**********subscribe point cloud from cam1, get right hand position from cam2 + transform matrix between cam1 and cam2*****************can run
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber_node")
  {
    // Subscribe to the point cloud topic
    pcl_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", 10, std::bind(&SubscriberNode::processPointCloud, this, std::placeholders::_1));

    // Subscribe to the Float32MultiArray topic
    float_array_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/Nuitrack/right_hand_position", 10, std::bind(&SubscriberNode::processFloatArray, this, std::placeholders::_1));

    // Create a publisher to publish the processed point cloud
    pcl_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/processed_point_cloud_topic", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr float_array_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;

  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Print the processed Float32MultiArray data
    RCLCPP_INFO(get_logger(), "right hand x: %.4f, y: %.4f, z: %.4f", right_hand_x, right_hand_y, right_hand_z);

    // Process point cloud data here
    pcl::CropBox<pcl::PointXYZRGB> crop_filter;
    crop_filter.setInputCloud(pcl_cloud);
    Eigen::Vector4f min_point(-1, -1, -1, 1);
    Eigen::Vector4f max_point(2, 2, 3, 1);
    crop_filter.setMin(min_point);
    crop_filter.setMax(max_point);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    crop_filter.filter(*processed_cloud);

    // Create a new point with xyz coordinates from Float32MultiArray
    pcl::PointXYZRGB new_point;
    new_point.x = right_hand_x;
    new_point.y = right_hand_y;
    new_point.z = right_hand_z;
    new_point.r = 255; // set the color to red
    new_point.g = 0;
    new_point.b = 0;

    // Push the new point into the processed point cloud
    processed_cloud->points.push_back(new_point);

    // Preserve the color of the point cloud
    processed_cloud->width = 1;
    processed_cloud->height = processed_cloud->points.size();
    sensor_msgs::msg::PointCloud2 processed_cloud_msg;
    pcl::toROSMsg(*processed_cloud, processed_cloud_msg);
    processed_cloud_msg.header = msg->header;

    // Publish the processed point cloud
    pcl_publisher->publish(processed_cloud_msg);
  }

  void processFloatArray(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // Process float array data here
    float_array_data_ = msg->data[0];
    float right_hand_x_cam2 = msg->data[0];
    float right_hand_y_cam2 = msg->data[1];
    float right_hand_z_cam2 = msg->data[2];

    // Define the transformation matrix from cam2 to cam1
    Eigen::Matrix4f cam2_to_cam1_transform;
    cam2_to_cam1_transform << 1, 0, 0, -0.1,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1;

    // Transform the right hand position from cam2 to cam1
    Eigen::Vector4f right_hand_pos_cam2(right_hand_x_cam2, right_hand_y_cam2, right_hand_z_cam2, 1);
    Eigen::Vector4f right_hand_pos_cam1 = cam2_to_cam1_transform * right_hand_pos_cam2;
    float right_hand_x_cam1 = right_hand_pos_cam1(0);
    float right_hand_y_cam1 = right_hand_pos_cam1(1);
    float right_hand_z_cam1 = right_hand_pos_cam1(2);

    // Use the processed data to process the point cloud data in processPointCloud() function
    right_hand_x = right_hand_x_cam1;
    right_hand_y = right_hand_y_cam1;
    right_hand_z = right_hand_z_cam1;
  }

  // Processed point cloud data
  float float_array_data_;
  float right_hand_x;
  float right_hand_y;
  float right_hand_z;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}


