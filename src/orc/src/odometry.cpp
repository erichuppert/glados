#include <cmath>
#include "orc/odometry.h"
#include "orc/status.h"
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
    last_time = status.utime_host;
    previous_left = uorc_encoder_get_position(&left_encoder, &status);
    previous_right = uorc_encoder_get_position(&right_encoder, &status);

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

void MotorStatus::update() {
    // Find the new positions
    //
    status = ost.get();
    // Avoid using repeated status
    // Mostly for the velocity, not really important for location
    //
    if (status.utime_host == last_time) {
        return;
    }

    int32_t left = uorc_encoder_get_position(&left_encoder, &status);
    int32_t right = uorc_encoder_get_position(&right_encoder, &status);

    double delta_left = (left-previous_left)*WHEEL_METERS_PER_TICK;
    double delta_right = (right-previous_right)*WHEEL_METERS_PER_TICK;

    double delta_distance = (delta_left + delta_right)/2.0;
    theta += (delta_left - delta_right)/WHEEL_BASE;
    theta = atan2(sin(theta), cos(theta)); // TO -PI,PI
    x += delta_distance*cos(theta);
    y += delta_distance*sin(theta);

    double actual_v_right = (double)(right - previous_right)*1000000.0/((double) status.utime_host-last_time);

    // Now for the velocity part of the odometry
    // The orcboard has information about the velocity in its status, but it seems inaccurate,
    //   so we'll use the information we have from the pose.
    //
    double v_left = SPEED(last_time,status.utime_host,previous_left,left);
    double v_right = SPEED(last_time,status.utime_host,previous_right,right);
    v = (v_left+v_right)/2.0;
    omega = (v_left-v_right)/WHEEL_BASE;

    last_time = status.utime_host;
    previous_left = left;
    previous_right = right;

    ROS_INFO("%d\t%.6f\t%.6f",right,theta,v_right);
}

void odometry(MotorStatus* mot) {
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    ros::Rate loop(ODO_FREQ);
    ROS_INFO("Starting to publish odometry");

    while (n.ok()) {
        // Update odometry
        //
        mot->update();

        // Because ROS is more general than 2D rotation, it uses Quaternions
        //
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(mot->theta);

        // Need to publish an odometry transform relative to the base link
        //
        current_time = ros::Time::now();
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = mot->x;
        odom_trans.transform.translation.x = mot->y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // Now let's publish the odometry message
        //
        nav_msgs::Odometry msg;
        msg.header.stamp = current_time;
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = mot->x;
        msg.pose.pose.position.y = mot->y;
        msg.pose.pose.position.z = 0;
        msg.pose.pose.orientation = odom_quat;

        msg.twist.twist.linear.x = mot->v*cos(mot->theta);
        msg.twist.twist.linear.y = mot->v*sin(mot->theta);
        msg.twist.twist.angular.z = mot->omega;

        odom_pub.publish(msg);

        loop.sleep();
    }
}
