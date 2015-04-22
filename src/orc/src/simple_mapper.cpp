// OccupancyGrid mapper, no localization
//
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_listener.h"

#include <string>
#include <cmath>
#include <cstring>

#define DEFAULT_WIDTH 300 // X direction
#define DEFAULT_HEIGHT 300 // Y direction
#define DEFAULT_X_MIN -10.0
#define DEFAULT_Y_MIN -10.0
#define DEFAULT_DELTA 0.01 // grid side length
#define DEFAULT_BASE_LINK "base_link"
#define DEFAULT_ODOM_FRAME "odom"

#define DEFAULT_P_WALL_G_WALL 0.8 // p(wall | wall)
#define DEFAULT_P_WALL_G_NWALL 0.2 // p(wall | no_wall)

#define DEFAULT_SCAN_TOPIC "scan"
#define DEFAULT_MAP_TOPIC "simple_map"

// Parameters
//
int width;
int height;
double x_min;
double y_min;

double delta;
std::string base_link;
std::string odom_frame;

double p_wall_g_wall;
double p_wall_g_nwall;

std::string scan_topic;
std::string map_topic;

#define INDEX(x,y) y*width+x
int8_t* map; // probability of occupied cell, 0-100; -1 for unknown

ros::Publisher map_pub;
tf::TransformListener transformer;

void update_map(const sensor_msgs::LaserScan::ConstPtr&);
void clear_line(geometry_msgs::PointStamped,double,double);
void update_cell(int,int);

void scan_handler(const sensor_msgs::LaserScan::ConstPtr& input) {
    nav_msgs::OccupancyGrid output;

    update_map(input);

    output.header.frame_id = odom_frame;
    output.header.stamp = ros::Time::now();
    output.info.map_load_time = ros::Time::now();
    output.info.resolution = delta;
    output.info.width = width;
    output.info.height = height;

    geometry_msgs::Pose origin;
    origin.position.x = x_min;
    origin.position.y = y_min;
    origin.position.z = 0;
    output.info.origin = origin;

    output.data.resize(width*height);
    for (int i = 0; i < width*height; i++) {
        output.data[i] = map[i];
    }

    map_pub.publish(output);
}

void update_map(const sensor_msgs::LaserScan::ConstPtr& input) {
    transformer.waitForTransform(odom_frame, input->header.frame_id, ros::Time(input->header.stamp), ros::Duration(5.0));

    geometry_msgs::PointStamped pt_robot;
    geometry_msgs::PointStamped pt_odom;
    int ticks = input->ranges.size();
    int ticks_0 = input->angle_min*ticks/(input->angle_min-input->angle_max);
    for (int i = 0; i <= ticks; i++) {
        double angle = (input->angle_max-input->angle_min)/ticks * (i-ticks_0);
        double range = input->ranges[i];

        pt_robot.header = input->header;
        pt_robot.point.x = range*cos(angle);
        pt_robot.point.y = range*sin(angle);
        pt_robot.point.z = 0;
        transformer.transformPoint(odom_frame,pt_robot,pt_odom);

        clear_line(pt_odom, range, angle);

        int x = 1./delta * (pt_odom.point.x-x_min);
        int y = 1./delta * (pt_odom.point.y-y_min);
        if (input->range_min < range && range < input->range_max) {
            update_cell(x,y);
        }
    }
}

#define DIST(x0,y0,x1,y1) sqrt(pow((x0)-(x1), 2) + pow((y0)-(y1), 2))
void clear_line(geometry_msgs::PointStamped pt_odom, double range, double angle) {
    double x_pt = pt_odom.point.x;
    double y_pt = pt_odom.point.y;
    int r_pt = 1./delta * (pt_odom.point.x-x_min);
    int c_pt = 1./delta * (pt_odom.point.y-y_min);
    double x = x_pt;
    double y = y_pt;
    for (double x = x_pt, y = y_pt; DIST(x_pt,y_pt,x,y) < range; x -= cos(angle)*delta, y -= sin(angle)*delta) {
        int r = 1./delta * (x-x_min);
        int c = 1./delta * (y-y_min);
        if (r != r_pt || c != c_pt) {
            double prior = map[INDEX(r,c)]/100.0;
            if (prior < 0) {
                map[INDEX(r,c)] = (1-p_wall_g_nwall) * 100;
            } else {
                double p_nwall_nwall = (1-prior) * (1-p_wall_g_nwall);
                double p_nwall = p_nwall_nwall + prior*(1-p_wall_g_wall);
                map[INDEX(r,c)] = (1-p_nwall_nwall/p_nwall) * 100;
            }
        }
    }
}

void update_cell(int x, int y) {
    double prior = map[INDEX(x,y)]/100.0;
    if (prior < 0) {
        map[INDEX(x,y)] = p_wall_g_wall*100;
    } else {
        double p_wall_and_wall = prior*p_wall_g_wall;
        double p_wall = p_wall_and_wall + (1-prior)*p_wall_g_nwall;
        map[INDEX(x,y)] = (p_wall_and_wall/p_wall) * 100;
    }
}

int main(int argc, char** argv) {
    // Initialize map
    // All unknown
    //
    memset(map,-1,sizeof(map));

    // Initialize ROS
    //
    ros::init(argc, argv, "simple_mapper");
    ros::NodeHandle nh;

    // Get parameters
    //
    nh.param("width", width, DEFAULT_WIDTH);
    nh.param("height", height, DEFAULT_HEIGHT);
    nh.param("x_min", x_min, DEFAULT_X_MIN);
    nh.param("y_min", y_min, DEFAULT_Y_MIN);
    nh.param("delta", delta, DEFAULT_DELTA);
    nh.param<std::string>("base_link", base_link, DEFAULT_BASE_LINK);
    nh.param<std::string>("odom_frame", odom_frame, DEFAULT_ODOM_FRAME);
    nh.param("p_wall_g_wall", p_wall_g_wall, DEFAULT_P_WALL_G_WALL);
    nh.param("p_wall_g_nwall", p_wall_g_nwall, DEFAULT_P_WALL_G_NWALL);
    nh.param<std::string>("scan_topic", scan_topic, DEFAULT_SCAN_TOPIC);
    nh.param<std::string>("map_topic", map_topic, DEFAULT_MAP_TOPIC);

    map = (int8_t*)malloc(sizeof(int8_t)*width*height);

    // Create subscriber to /scan topic
    //
    ros::Subscriber scan_sub = nh.subscribe(scan_topic, 1, scan_handler);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);
}
