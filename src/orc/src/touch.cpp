#include "ros/ros.h"
#include "orc/TouchState.h"
#include "orc/touch.h"

bool DigitalInput::getValue() {
    uorc_status_t status = ost->get();
    bool v;
    if (port < 8) {
        v = ((status.simple_digital_values&(1<<port))!=0);
    } else {
        v = (status.fast_digital_config[port-8]!=0);
    }

    return v!=invert;
}

void monitor_touch(OrcStatus* ost) {
    DigitalInput gripper(ost, PORT_GRIPPER, true, true);

    ros::NodeHandle nh;
    ros::Publisher touch_pub = nh.advertise<orc::TouchState>("touch", 50);
    ros::Rate loop(JOINT_FREQ);

    while (nh.ok()) {
        orc::TouchState msg;
        msg.gripper = gripper.getValue();
        touch_pub.publish(msg);
        loop.sleep();
    }
}
