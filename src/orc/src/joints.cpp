#include "orc/uorc.h"
#include "orc/JointSet.h"
#include "orc/status.h"
#include "ros/ros.h"
#include "orc/odometry.h"

#define JOINT_FREQ 30

void joints(MotorStatus* mot) {
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",50);
    ros::Time current_time;
    ros::Rate loop(JOINT_FREQ);
    ROS_INFO("Starting to publish joint states");

    while (n.ok()) {
        // Get latest status
        //
        mot->update();
        current_time = ros::Time::now();
        msg.header.stamp = current_time;

        msg.name.push_back("base_to_lwheel");
        msg.position.push_back(mot->left_angle);

        msg.name.push_back("base_to_rwheel");
        msg.position.push_back(mot->right_angle);

        loop.sleep();
    }
}
