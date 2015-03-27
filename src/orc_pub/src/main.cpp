#include <thread>
#include "orc_pub/odometry.h"
#include "orc_pub/status.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    uorc_t *uorc = uorc_create();
    ros::init(argc,argv,"GLaDOS uORC Talker");

    // Start registering statuses from the orcboard
    //
    OrcStatus ost(uorc);
    std::thread statusT(updateStatus,&ost);
    ros::Duration(1.0).sleep(); // Make sure we get some status

    MotorStatus mot(ost);
    std::thread odoT(odometry,&mot);

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
