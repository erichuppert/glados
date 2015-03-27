#include "orc/uorc.h"
#include "orc/status.h"
#include "ros/ros.h"
#include "orc/odometry.h"
#include "sensor_msgs/JointState.h"
#include "orc/JointSet.h"
#include "orc/joints.h"

// Servos
//

Servo wrist(WRIST_INDEX,
            WRIST_MIN_PWM, WRIST_MAX_PWM,
            WRIST_ALPHA, WRIST_BETA,
            WRIST_SPEED);

Servo shoulder(SHOULDER_INDEX,
               SHOULDER_MIN_PWM, SHOULDER_MAX_PWM,
               SHOULDER_ALPHA, SHOULDER_BETA,
               SHOULDER_SPEED);

Servo gripper(GRIPPER_INDEX,
              GRIPPER_MIN_PWM, GRIPPER_MAX_PWM,
              GRIPPER_ALPHA, GRIPPER_BETA,
              GRIPPER_SPEED);

double Servo::theta(int pwm) {
    pwm == 0 && (pwm = min_pwm);
    return (pwm-beta)/alpha;
}

int Servo::pwm(double theta) {
    int pwm = alpha*theta+beta;
    assert(pwm >= min_pwm && pwm <= max_pwm);
    return pwm;
}

void Servo::setPosition(int pwm, uorc_t* uorc) {
    assert(pwm == 0 || (pwm >= min_pwm && pwm <= max_pwm));
    {
        AutoLock l(&mtx);
        if (!done) return;
        done = false;
    }
    target_pwm = pwm; // 0 is idle
    current_thread = new std::thread(std::ref(*this), uorc);
}

void Servo::setAngle(double theta, uorc_t* uorc) {
    setPosition(pwm(theta),uorc);
}

bool Servo::isDone() {
    AutoLock l(&mtx);
    return done;
}

void Servo::operator () (uorc_t* uorc) {
    int delta_pwm = abs(current_pwm - target_pwm);
    uint8_t buf[] = {(uint8_t)index,
                     (uint8_t)(target_pwm==0?
                               FAST_DIGIO_MODE_OUT:FAST_DIGIO_MODE_SERVO),
                     (uint8_t)((target_pwm>>24)&0xff),
                     (uint8_t)((target_pwm>>16)&0xff),
                     (uint8_t)((target_pwm>>8 )&0xff),
                     (uint8_t)((target_pwm>>0 )&0xff)
    };
    uorc_response* resp = nullptr;
    do {
        if(resp != nullptr) uorc_response_destroy(resp);
        resp = uorc_command(uorc, 0x7000, buf, sizeof(buf), -1);
    } while(!resp->valid);
    uorc_response_destroy(resp);
    ros::Duration(delta_pwm/speed).sleep();
    mtx.lock();
    current_pwm = target_pwm;
    done = true;
    mtx.unlock();
}

double Servo::current() {
    AutoLock l(&mtx);
    return theta(current_pwm);
}

OrcStatus* g_status;

void joints(OrcStatus* ost, MotorStatus* mot) {
    g_status = ost;
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states",50);
    ros::Subscriber joint_sub = n.subscribe("joint_set", 10, joint_set);
    ros::Time current_time;
    ros::Rate loop(JOINT_FREQ);

    // Set the arm to idle
    //
    shoulder.setPosition(0, ost->uorc());
    wrist.setPosition(0, ost->uorc());
    gripper.setPosition(0, ost->uorc());

    while (n.ok()) {
        // Get latest status
        //
        current_time = ros::Time::now();
        sensor_msgs::JointState msg;
        msg.header.stamp = current_time;

        // Wheel position
        //
        msg.name.push_back("base_to_lwheel");
        msg.position.push_back(mot->left_angle);

        msg.name.push_back("base_to_rwheel");
        msg.position.push_back(mot->right_angle);

        // Arm position
        //
        msg.name.push_back("base_to_shoulder");
        msg.position.push_back(shoulder.current());

        msg.name.push_back("shoulder_to_wrist");
        msg.position.push_back(wrist.current());

        msg.name.push_back("wrist_to_gripper");
        msg.position.push_back(gripper.current());

        joint_pub.publish(msg);

        ros::spinOnce();
        loop.sleep();
    }
}


void joint_set(const orc::JointSet::ConstPtr& msg) {
    if(msg->joint_name == "base_to_shoulder") {
        shoulder.setAngle(msg->angle, g_status->uorc());
    } else if(msg->joint_name == "shoulder_to_wrist") {
        wrist.setAngle(msg->angle, g_status->uorc());
    } else if(msg->joint_name == "wrist_to_gripper") {
        gripper.setAngle(msg->angle, g_status->uorc());
    } else {
        ROS_WARN("Invalid joint name");
    }
}
