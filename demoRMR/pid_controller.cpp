#include "pid_controller.h"
#include <cstdlib>

PIDController::PIDController(double kp, double ki, double kd, double dt, double deadband)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt), deadband_(deadband), integral_(0.0), prev_error_(0.0) {}

double PIDController::Update(double setpoint, double measured_value) {
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

    return output;
}
