#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt, double deadband);

    double Update(double setpoint, double measured_value);

private:
    double kp_;
    double ki_;
    double kd_;
    double dt_;
    double deadband_;
    double integral_;
    double prev_error_;
    double prev_output_;
};

#endif /* PID_CONTROLLER_H_ */
