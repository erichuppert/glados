#ifndef ORC_PUB_ODOMETRY_H
#define ORC_PUB_ODOMETRY_H

#include "orc_utils/uorc.h"
#include "orc_pub/status.h"
#include <cmath>

#define TICKS_PER_REVOLUTION 4 // TICKS/REV
#define GEAR_RATIO
#define WHEEL_RADIUS 0.0625 // M/(REV*2*pi)
#define WHEEL_BASE 0.43 // M
#define WHEEL_METERS_PER_TICK (WHEEL_RADIUS)*(2*M_PI)/(TICKS_PER_REVOLUTION) // M/TICK

#define ODO_FREQ 30 //HZ

class MotorStatus {
private:
    uint32_t previous_left;
    uint32_t previous_right;

    OrcStatus& ost;

    uorc_encoder_t left_encoder;
    uorc_encoder_t right_encoder;

    uorc_status_t status;
public:
    double v;
    double omega;

    double x;
    double y;
    double theta;

    MotorStatus(OrcStatus&);

    // Back to the origin
    //
    void resetOdom();

    void update();
};

void odometry(MotorStatus*);
#endif
