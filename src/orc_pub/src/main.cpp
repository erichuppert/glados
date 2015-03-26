#include <thread>
#include <mutex>
#include "orc_utils/uorc.h"
#include "orc_pub/odometry.h"
#include "orc_pub/status.h"

#define STATUS_RATE 40

OrcStatus::OrcStatus(uorc_t* uorc):
    uorc(uorc)
{}

void OrcStatus::set() {
    uorc_get_status(uorc, &status);
    mtx.lock();
    status_copy = status;
    mtx.unlock();
}

uorc_status_t& OrcStatus::get() {
    mtx.lock();
    uorc_status_t ret = status_copy;
    mtx.unlock();
    return ret;
}

uorc_t* OrcStatus::uorc() {
    return uorc;
}

void updateStatus(OrcStatus& ost) {
    ros::Rate status_update(STATUS_RATE);
    while(ros::ok()) {
        ost.set();
        status_update.sleep();
    }
}

int main(int argc, char *argv[])
{
    uorc_t *uorc = uorc_create();
    uorc_status status;
    ros::init(argc,argv,"GLaDOS uORC Talker");

    // Start registering statuses from the orcboard
    //
    std::thread statusT(updateStatus,uorc,&status);
    ros::sleep(1); // Make sure we get some status

    MotorStatus mot(ost);
    std::thread odoT(odometry,ost,mot);

    statusT.join();
    // If we are no longer getting statuses, but haven't shutdown,
    // there's most likely a problem with the orcboard, so we're just shutting down.
    //
    if (ros::ok()) {
        ros::shutdown();
    }

    odoT.join();

    uorc_destroy(uorc);
}
