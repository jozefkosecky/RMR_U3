#ifndef P_CONTROLLER_MOVEMENT_H
#define P_CONTROLLER_MOVEMENT_H
#include <math.h>
#include <iostream>
#include <cmath>

class PControllerMovement {
public:
    PControllerMovement(double kp, double dt, int rampRate);

    double Update(double setpoint, double measured_value);
    void UpdateOutputToZero();

private:
    double kp_;
    double dt_;
    double prev_output_;
    int rampRate_;
};

#endif /* PID_CONTROLLER_H_ */
