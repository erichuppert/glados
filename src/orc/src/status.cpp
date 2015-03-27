#include "orc/status.h"
#include "ros/ros.h"

OrcStatus::OrcStatus(uorc_t* uorc):
    _uorc(uorc),
    status_received(false)
{}

void OrcStatus::set() {
    uorc_get_status(_uorc, &status);
    status_received = true;
    mtx.lock();
    status_copy = status;
    mtx.unlock();
}

uorc_status_t OrcStatus::get() {
    AutoLock l(&mtx);
    return status_copy;
}

uorc_t* OrcStatus::uorc() {
    return _uorc;
}

void updateStatus(OrcStatus* ost) {
    ros::Rate status_update(STATUS_RATE);
    while(ros::ok()) {
        ost->set();
        status_update.sleep();
    }
}