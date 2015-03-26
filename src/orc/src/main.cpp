#include <thread>
#include "orc/odometry.h"
#include "orc/status.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    uorc_t *uorc = uorc_create();
    ros::init(argc,argv,"glados_uorc");
    ros::NodeHandle first;

    // Start registering statuses from the orcboard
    //
    OrcStatus ost(uorc);
    std::thread statusT(updateStatus,&ost);
    ROS_INFO("Waiting on first status update");
    while (!ost.status_received) {
        ros::Duration(1.0).sleep(); // Make sure we get some status
    }
    ROS_INFO("First status update received");

    MotorStatus mot(ost);
    std::thread odoT(odometry,&mot);

    statusT.join();
    // If we are no longer getting statuses, but haven't shutdown,
    // there's most likely a problem with the orcboard, so we're just shutting down.
    //
    if (ros::ok()) {
        ROS_FATAL("Unknown orcBoard error, shutting down.");
        ros::shutdown();
    }

    odoT.join();

    uorc_destroy(uorc);
}
