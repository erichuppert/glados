#include "orc/pid.h"
#include "orc/odometry.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define min(X,Y) ((X)>(Y)?(Y):(X))
#define max(X,Y) ((X)>(Y)?(X):(Y))
#define CAP(signal) min(max((signal),(-1)),1)
#define EPSILON 0.001
#define sign(s) ((s)>0?(1):(-1))
#define ABS(A) (sign(A)*(A))

PID left_wheel(0.75,18.0,-0.0005);
PID right_wheel(0.75,18.0,-0.0005);

void PID::setObjective(double _objective) {
    mtx.lock();
    if (objective*_objective < 0) {
        i = 0;
    }
    objective = _objective;
    previous_timestamp = -1;
    mtx.unlock();
}

double PID::control(double actual, double timestamp) {
    mtx.lock();
    double error = objective-actual;
    mtx.unlock();
    double deriv = 0;
    if (ABS(objective) <= EPSILON && ABS(error) <= 0.05) {
        i = 0;
    }
    if (previous_timestamp != -1) {
        double dt = timestamp-previous_timestamp;
        i += (error)*dt/2.0;
        deriv = (error-previous_error)/dt;
    }
    mtx.lock();
    previous_timestamp = timestamp;
    previous_error = error;
    mtx.unlock();

    return p_gain*error + i_gain*i + d_gain*deriv;
}

void controller(OrcStatus* ost, MotorStatus* mot) {
    uorc_motor_t left_motor = {
        .uorc = ost->uorc(),
        .port = 1,
        .invert = 0
    };
    uorc_motor_t right_motor = {
        .uorc = ost->uorc(),
        .port = 0,
        .invert = 1
    };
    ros::Time current_time;
    ros::Rate loop(ODO_FREQ);

    // Start stopped
    //
    left_wheel.setObjective(0);
    right_wheel.setObjective(0);

    ros::NodeHandle n;
    ros::Subscriber vel_sub = n.subscribe("/cmd_vel",10,update_objective);

    while(ros::ok()) {
        current_time = ros::Time::now();
        double left_signal = left_wheel.control(mot->v_left, current_time.toSec());
        left_signal = CAP(left_signal);
        double right_signal = right_wheel.control(mot->v_right, current_time.toSec());
        right_signal = CAP(right_signal);
        uorc_motor_set_pwm(&left_motor,left_signal);
        uorc_motor_set_pwm(&right_motor,right_signal);
        ros::spinOnce();
        loop.sleep();
    }
}

void update_objective(const geometry_msgs::Twist::ConstPtr& msg) {
    double v = MAX(MIN(msg->linear.x, MAX_SPEED),MIN_SPEED);
    double omega = (msg->angular.z)*-1;
    double v_diff = MAX(MIN(omega*WHEEL_BASE/2.0;,MAX_SPEED),MIN_SPEED);
    left_wheel.setObjective(v+v_diff);
    right_wheel.setObjective(v-v_diff);
}
