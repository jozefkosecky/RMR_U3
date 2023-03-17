#include "p_controller_movement.h"
#include <math.h>
#include <iostream>
#include <cmath>


PControllerMovement::PControllerMovement(double kp, double dt, int rampRate)
    : kp_(kp), dt_(dt), prev_output_(0.0), rampRate_(rampRate) {}

double PControllerMovement::Update(double setpoint, double measured_value) {
    // Calculate the error
    double error = setpoint - measured_value;

    // Calculate the proportional term
    double proportional = kp_ * error;

    // Calculate the control output
    double output = proportional;

    // Apply the ramp rate
    output = std::max(prev_output_ - rampRate_ * dt_, std::min(prev_output_ + rampRate_ * dt_, output));
    prev_output_ = output;

    return output;
}

void PControllerMovement::UpdateOutputToZero(){
    prev_output_ = 0.0;
}
