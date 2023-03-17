#include "p_controller_rotation.h"
#include <cstdlib>

PControllerRotation::PControllerRotation(double kp, double dt)
    : kp_(kp), dt_(dt) {}

double PControllerRotation::Update(double setpoint, double measured_value) {
    // Calculate the error
    double error = setpoint - measured_value;

    // Calculate the proportional term
    double proportional = kp_ * error;

    // Calculate the control output
    double output = proportional;

    return output;
}
