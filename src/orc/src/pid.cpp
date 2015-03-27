#include "orc/pid.h"

void setObjective(double _objective) {
    objective = _objective;
    i = 0;
    previous_error = 0;
    previous_timestamp = -1;
}

double PID::control(double error, double timestamp) {
    double error = objective-actual;
    double deriv = 0;
    if (previous_timestamp != -1) {
        double dt = timestsamp-previous_timestamp;
        i += (previous_error+error)*dt/2.0;
        deriv = (error-previous_error)/dt;
    }
    previous_timestamp = timestamp;

    return p_gain*error + i_gain*i + d_gain*deriv;
}
