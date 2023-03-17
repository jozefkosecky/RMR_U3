#ifndef P_CONTROLLER_ROTATION_H_
#define P_CONTROLLER_ROTATION_H_

class PControllerRotation {
public:
    PControllerRotation(double kp, double dt);

    double Update(double setpoint, double measured_value);

private:
    double kp_;
    double dt_;
};

#endif /* P_CONTROLLER_ROTATION_H_ */
