#ifndef ORC_JOINTS_H
#define ORC_JOINTS_H

#define JOINT_FREQ 30

#include <cmath>
#include "orc/odometry.h"

// PWM = alpha*theta + beta
// theta = (PWM-beta)/alpha
//
// Shoulder
//
#define SHOULDER_INDEX 5
#define SHOULDER_MIN_PWM 550
#define SHOULDER_MAX_PWM 2450

#define SHOULDER_ALPHA 445.63
#define SHOULDER_BETA 1400

#define SHOULDER_SPEED M_PI/4

// Wrist
//
#define WRIST_INDEX 4
#define WRIST_MIN_PWM 600
#define WRIST_MAX_PWM 2400

#define WRIST_ALPHA -477.46
#define WRIST_BETA 1800

#define WRIST_SPEED M_PI/4

// Gripper
//
#define GRIPPER_INDEX 2
#define GRIPPER_MIN_PWM 1200
#define GRIPPER_MAX_PWM 2150

#define GRIPPER_ALPHA 911.52
#define GRIPPER_BETA 1004.55

#define GRIPPER_SPEED 0.1

#define DOOR_INDEX 6
#define DOOR_MIN_PWM 1700
#define DOOR_MAX_PWM 2225

#define DOOR_ALPHA 334.23
#define DOOR_BETA 1700

#define DOOR_SPEED M_PI/2

#include "orc/uorc.h"
#include <thread>
#include <mutex>

#define FAST_DIGIO_MODE_IN 1
#define FAST_DIGIO_MODE_OUT 2
#define FAST_DIGIO_MODE_SERVO 3
#define FAST_DIGIO_MODE_SLOW_PWM 4

#include "orc/JointSet.h"

class Servo {
private:
    const int max_pwm;
    const int min_pwm;
    const double alpha;
    const double beta;
    double speed; // pwms/sec
    int target_pwm;
    std::mutex mtx;
    bool done;
    int current_pwm;

    void goToPosition(uorc_t* uorc, int target_pwm);
public:
    const int index;
    std::thread* current_thread;

    Servo(int index,
          int min_pwm, int max_pwm,
          double alpha, double beta,
          double omega) // radians/sec
        : index(index),
        min_pwm(min_pwm),
        max_pwm(max_pwm),
        alpha(alpha),
        beta(beta),
        current_pwm(0),
        done(true)
        {
            speed = abs(alpha*omega);
        }

    double theta(int pwm);
    int pwm(double theta);
    void setPosition(int pwm, uorc_t* uorc);
    void setAngle(double theta, uorc_t* uorc);
    bool isDone();
    void operator () (uorc_t* uorc);
    double current();
};

void joints(OrcStatus*,MotorStatus*);

void joint_set(const orc::JointSet::ConstPtr& msg);

#endif
