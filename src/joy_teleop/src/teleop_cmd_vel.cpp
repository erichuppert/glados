#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "orc/JointSet.h"

class Teleop
{
public:
    Teleop();
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Publisher joint_pub_;

};

Teleop::Teleop():
    linear_(1),
    angular_(2)
{

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
    joint_pub_ = nh_.advertise<orc::JointSet>("/joint_set", 100);
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    orc::JointSet joint;
    vel.angular.z = a_scale_*joy->axes[angular_];
    vel.linear.x = l_scale_*joy->axes[linear_];
    vel_pub_.publish(vel);

    if (joy->buttons[0]) { // Find
        joint.joint_name = "base_to_shoulder";
        joint.angle = -1.3;
        joint_pub_.publish(joint);

        joint.joint_name = "shoulder_to_wrist";
        joint.angle = 0.8;
        joint_pub_.publish(joint);

        joint.joint_name = "wrist_to_gripper";
        joint.angle = 0.8;
        joint_pub_.publish(joint);
    } else if(joy->buttons[2]) { // Grip
        joint.joint_name = "base_to_shoulder";
        joint.angle = -1.3;
        joint_pub_.publish(joint);

        joint.joint_name = "shoulder_to_wrist";
        joint.angle = 0.8;
        joint_pub_.publish(joint);

        joint.joint_name = "wrist_to_gripper";
        joint.angle = 0.55;
        joint_pub_.publish(joint);
    } else if(joy->buttons[3]) { // Pick-up
        joint.joint_name = "base_to_shoulder";
        joint.angle = 2.0;
        joint_pub_.publish(joint);

        joint.joint_name = "shoulder_to_wrist";
        joint.angle = 1.24;
        joint_pub_.publish(joint);

        joint.joint_name = "wrist_to_gripper";
        joint.angle = 0.55;
        joint_pub_.publish(joint);
    } else if(joy->buttons[1]) { // Drop
        joint.joint_name = "base_to_shoulder";
        joint.angle = 2.0;
        joint_pub_.publish(joint);

        joint.joint_name = "shoulder_to_wrist";
        joint.angle = 1.24;
        joint_pub_.publish(joint);

        joint.joint_name = "wrist_to_gripper";
        joint.angle = 0.8;
        joint_pub_.publish(joint);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_teleop");
    Teleop teleop;

    ros::spin();
}
