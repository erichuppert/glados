#ifndef ORC_PID_H
#define ORC_PID_H

#include "status.h"
#include "odometry.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <mutex>

class PID {
private:
    const double p_gain;
    const double i_gain;
    const double d_gain;
    double previous_timestamp;
    double i;
    double previous_error;
    double objective;
    std::mutex mtx;
public:
    PID(double p, double i, double d)
        :p_gain(p),
        i_gain(i),
        d_gain(d),
        i(0),
        previous_timestamp(-1)
    {}

    void setObjective(double _objective);
    double control(double actual, double timestamp);
};

void controller(OrcStatus*,MotorStatus*);

void update_objective(const geometry_msgs::Twist::ConstPtr& msg);

#endif
