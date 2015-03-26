#include <cmath>
#include "orc_pub/odometry.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"


MotorStatus::MotorStatus(OrcStatus& ost): ost(ost) {
    left_encoder = {
        .uorc = ost.uorc(),
        .port = 0,
        .invert = 0
    };
    right_encoder = {
        .uorc = ost.uorc(),
        .port = 1,
        .invert = 1
    };

    // Find the initial positions
    //
    status = ost.get();
    previous_left = uorc_encoder_get_position(&left_encoder, *status);
    previous_right = uorc_encoder_get_position(&right_encoder, *status);

    // Start at the origin
    //
    resetOdom();
}

// Back to the origin
//
void MotorStatus::resetOdom() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
}

void MotorStatus::udpate() {
    // Find the new positions
    //
    status = ost.get();
    uint32_t left = uorc_encoder_get_position(&left_encoder, &status);
    uint32_t right = uorc_encoder_get_position(&right_encoder, &status);

    uint32_t delta_left = left-previous_left;
    uint32_t delta_right = right-previous_right;

    double delta_distance = WHEEL_METERS_PER_TICK*(delta_left + delta_right)/2.0;
    theta += (delta_left - delta_right)/WHEEL_BASE;
    theta = atan2(sin(theta), cos(theta)); // TO -PI,PI
    x += delta_distance*cos(theta);
    y += delta_distance*sin(theta);

    previous_left = left;
    previous_right = right;

    // Now for the velocity part of the odometry
    //
    uint32_t v_left = uorc_encoder_get_velocity(&left_encoder, &status);
    uint32_t v_right = uorc_encoder_get_velocity(&right_encoder, &status);
    double v = WHEEL_METERS_PER_TICK*(v_left+v_right)/2.0;
    double omega = (v_left-v_right)/WHEEL_BASE;
}

void odometry(OrcStatus& ost, MotorStatus &mot) {
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    ros::Rate loop(ODO_FREQ);
    ROS_INFO("Starting to publish odometry");

    while (n.ok()) {
        // Update odometry
        //
        mot.update();

        // Because ROS is more general than 2D rotation, it uses Quaternions
        //
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(mot.omega);

        // Need to publish an odometry transform relative to the base link
        //
        current_time = ros::Time::now();
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = mot.x;
        odom_trans.transform.translation.x = mot.y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.translation.x = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // Now let's publish the odometry message
        //
        nav_msgs::Odometry msg;
        msg.header.stamp = current_time;
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = mot.x;
        msg.pose.pose.position.y = mot.y;
        msg.pose.pose.position.z = 0;
        msg.pose.pose.orientation = odom_quat;

        msg.twist.linear.x = mot.v*cos(mot.theta);
        msg.twist.linear.y = mot.v*sin(mot.theta);
        msg.twist.angular.z = mot.omega;

        odom_pub.publish(msg);

        loop.sleep();
    }
}
