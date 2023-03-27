#include "rclcpp/rclcpp.hpp"
#include <limits>
#include <cmath>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/common/common.h>
//#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"

class MinimumDistanceNode : public rclcpp::Node {
public:
    MinimumDistanceNode() : Node("minimum_distance_node") {
        
        //create a subscriber to get pointcloud data from topic
        //create a publisher to send min_distance after pointcloud data processing
        pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("minimum_distance_topic", 100);
        //pub_ = create_publisher<std_msgs::msg::Float32>("minimum_distance_topic", 100);
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", //subscription topic can be set here
            1, // QoS history depth
            std::bind(&MinimumDistanceNode::topic_callback, this, std::placeholders::_1)
            );
    }

private:
    //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    //all pointcloud data processing are doing in this function
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        //get xyz coordinates of all pointcloud
        const size_t number_of_points = msg->height * msg->width;
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        
        //set a fixed point to compare with all pointcloud point, and get the min_distance
        //unit is meter
        //double distance=0,fixed_X=0,fixed_Y=0,fixed_Z=0,min_distance=1000;
        //double min_x,min_y,min_d;
        float distance=0,fixed_X=0,fixed_Y=0,fixed_Z=0,min_distance=1000;
        float min_x,min_y,min_d;
        for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
          double x = *iter_x;
          double y = *iter_y;
          double d = *iter_z;
          distance= sqrt(pow(fixed_X-x,2)+pow(fixed_Y-y,2)+pow(fixed_Z-d,2));
          if (distance<min_distance){
          min_distance=distance;
          min_x = x;
          min_y = y;
          min_d = d;
          }
        }
        //display min_distance in Terminal
        RCLCPP_INFO(this->get_logger(), "Min_distance is %.4lf",min_distance);
        RCLCPP_INFO(this->get_logger(), "Min_distance Point xyd %.4lf %.4lf %.4lf",min_x, min_y, min_d);
        //publish this min_distance
        //auto msg_out = std_msgs::msg::Float32();
        auto msg_out = std_msgs::msg::Float32MultiArray();
        msg_out.data = {min_distance, min_x, min_y, min_d};
        pub_->publish(msg_out);
                    
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimumDistanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
