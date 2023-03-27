/*Here is only the experimental area*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <time.h>
#include <chrono>
#include <ctime>
#include <pcl/point_cloud.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
//#include <pcl_conversions/pcl_conversions.h>

//#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std;
using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
 
class PclSub : public rclcpp::Node
{
 
public:
    PclSub(std::string name)
        : Node(name)
    {
        using std::placeholders::_1;
        sub_novel = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/camera/depth/color/points", 0.5, std::bind(&PclSub::topic_callback, this, std::placeholders::_1));
        //Here add the topic,that you want to subscribe 
    }
 
private:
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_novel;
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const size_t number_of_points = msg->height * msg->width;
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        double distance=0,fixed_X=0,fixed_Y=0,fixed_Z=0,min_distance=1000;
        for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            double x = *iter_x;
            double y = *iter_y;
            double z = *iter_z;
            distance= sqrt(pow(fixed_X-x,2)+pow(fixed_Y-y,2)+pow(fixed_Z-z,2));
            if (distance<min_distance){
            min_distance=distance;
            }
            
        }
        RCLCPP_INFO(this->get_logger(), "Min_distance is %.4lf",min_distance);

        //get time stamp
        /*std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::time_t timestamp =std::chrono::system_clock::to_time_t(now);*/
        //auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        //auto sec_since_epoch = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();

        /*
        //save min_distance in a .txt file
        ofstream outfile;
        outfile.open("test.txt", ios::app);
        outfile << "min_distance";
        outfile << " ";
        outfile << "value=";
        outfile << min_distance;
        //outfile << " ";
        //outfile << millisec_since_epoch; 
        outfile << "\n";
        //outfile << endl;
        outfile.close();
        */

        //save min_distance in a .csv file
        /*ofstream outfile;
        outfile.open("test.csv",ios::out | ios::trunc | ios::app);
        outfile << "min_distance"
        << ","
        << "timestamp"
        << std::endl;
        */
        


    }

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PclSub>("pclsub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
   
}