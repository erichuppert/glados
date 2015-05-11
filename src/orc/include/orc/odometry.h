#ifndef ORC_PUB_ODOMETRY_H
#define ORC_PUB_ODOMETRY_H

#include "orc/uorc.h"
#include "orc/status.h"
#include <cmath>

#define ENCODER_RESOLUTION    2000                                           // TICKS/(MOTOR REV)
#define WHEEL_RADIUS          0.0625                                         // M/(REV*2*pi)
#define GEAR_RATIO            65.5                                           // (MOTOR REV)/REV
#define TICKS_PER_REVOLUTION  ENCODER_RESOLUTION*GEAR_RATIO                  // TICKS/REV
#define WHEEL_METERS_PER_TICK (WHEEL_RADIUS)*(2*M_PI)/(TICKS_PER_REVOLUTION) // M/TICK
#define WHEEL_BASE            0.381                                          // M

#define ODO_FREQ              30                                             // UPDATES/SEC

#define SPEED(T0,T1,TICK0,TICK1) (((double)TICK1-TICK0)*WHEEL_METERS_PER_TICK*1000000.0/((double)T1-T0)) // Takes time in microseconds

#define ANGLE(TICKS) (((double)TICKS)/(TICKS_PER_REVOLUTION) * 2 * M_PI)

class MotorStatus {
private:
    int32_t previous_left;
    int32_t previous_right;
    int64_t last_time;

    OrcStatus& ost;

    uorc_encoder_t left_encoder;
    uorc_encoder_t right_encoder;

    uorc_status_t status;
public:
    double v_left;
    double v_right;
    double v;
    double omega;

    double x;
    double y;
    double theta;

    double left_angle;
    double right_angle;

    MotorStatus(OrcStatus&);

    // Back to the origin
    //
    void resetOdom();

    void update();
};

void odometry(MotorStatus*);
#endif
