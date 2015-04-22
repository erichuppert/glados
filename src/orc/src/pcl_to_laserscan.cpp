#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "pcl_conversions/pcl_conversions.h"

#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"

#include <string>
#include <cmath>
#include <cstring>

#define DEFAULT_PCL_TOPIC "camera/depth/points"
#define DEFAULT_SCAN_TOPIC "pcl_scan"
#define DEFAULT_BASE_LINK "base_link"

std::string pcl_topic;
std::string scan_topic;
std::string base_link;

tf::TransformListener *tf_listener;
ros::Publisher scan_pub;

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& input) {
    ros::NodeHandle nh;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*input, pcl_pc);
    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl_pc.header.stamp = 0;
    pcl::fromPCLPointCloud2(pcl_pc,cloud);

    tf_listener->waitForTransform(base_link, pcl_pc.header.frame_id, ros::Time(pcl_pc.header.stamp), ros::Duration(5.0));

    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    pcl_ros::transformPointCloud(base_link, cloud, cloud_out, *tf_listener);
    sensor_msgs::LaserScan output;

    output.header.stamp = (*input).header.stamp;
    output.header.frame_id = base_link;

    unsigned int num_readings = 300;
    double laser_frequency = 10000;

    output.angle_min = -M_PI;
    output.angle_max = M_PI;
    output.angle_increment = M_PI*2/num_readings;
    output.time_increment = (1./laser_frequency) / (num_readings);
    output.range_min = 0.1;
    output.range_max = 4.5;
    output.ranges.resize(num_readings);
    double z_min = -0.05;
    double z_max = 0.1;
    int range_num[num_readings];
    memset(range_num, 0x0, sizeof(range_num));
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    for (it = cloud_out.begin(); it != cloud_out.end(); it++) {
        if (it->z > z_min && it->z < z_max) {
            double angle = atan2(it->y,it->x);
            int bucket = (angle-output.angle_min)*num_readings/(output.angle_max-output.angle_min);
            double range = sqrt(it->x*it->x + it->y*it->y);
            if (range_num[bucket] == 0) {
                output.ranges[bucket] = 0;
            }
            output.ranges[bucket] += range;
            range_num[bucket]++;
        }
    }
    for(int i = 0; i < num_readings; i++) {
        if (range_num[i]==0) {
            output.ranges[i] = 0;
        } else {
            output.ranges[i] /= (double)range_num[i];
        }
    }
    scan_pub.publish(output);
}

int main(int argc, char** argv) {
    // Initialize ROS
    //
    ros::init(argc, argv, "pointcloud2_to_laserscan");
    ros::NodeHandle nh;

    // Get parameters
    //
    nh.param<std::string>("pcl_topic", pcl_topic, DEFAULT_PCL_TOPIC);
    nh.param<std::string>("scan_topic", scan_topic, DEFAULT_SCAN_TOPIC);
    nh.param<std::string>("base_link", base_link, DEFAULT_BASE_LINK);

    // Create ROS subscriber for input point cloud
    //
    ros::Subscriber sub = nh.subscribe(pcl_topic, 1, cloud_cb);
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic, 1);
    tf_listener = new tf::TransformListener();

    // Spin
    //
    ros::spin();
}
