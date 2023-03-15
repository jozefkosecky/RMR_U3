#include <iostream>
#include <cmath>

// Define the PID controller class
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt), integral_(0.0), prev_error_(0.0) {}

    double Update(double setpoint, double measured_value) {
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
        double output = proportional + integral_ + derivative;

        return output;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double dt_;
    double integral_;
    double prev_error_;
};
