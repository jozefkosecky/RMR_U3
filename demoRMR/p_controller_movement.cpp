#include "p_controller_movement.h"
#include <math.h>
#include <iostream>
#include <cmath>


PControllerMovement::PControllerMovement(double kp, double ki, double kd, double dt, double deadband, int rampRate)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt), deadband_(deadband), integral_(0.0), prev_error_(0.0), prev_output_(0.0), rampRate_(rampRate) {}

double PControllerMovement::Update(double setpoint, double measured_value) {
    // Calculate the error
    double error = setpoint - measured_value;


    // Calculate the proportional term
    double proportional = kp_ * error;

    // Calculate the integral term
    integral_ += ki_ * error * dt_;

    // Calculate the derivative term
    double derivative = kd_ * (error - prev_error_) / dt_;
    prev_error_ = error;

    // Calculate the control output
//    double output = proportional + integral_ + derivative;
    double output = proportional + 0 + 0;

    // Apply the ramp rate
    output = std::max(prev_output_ - rampRate_ * dt_, std::min(prev_output_ + rampRate_ * dt_, output));
    prev_output_ = output;

    return output;
}

void PControllerMovement::UpdateOutputToZero(){
    prev_output_ = 0.0;
}
