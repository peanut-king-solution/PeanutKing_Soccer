#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
  PIDController(double kp, double ki, double kd);
  double update(double currentValue);

// variables
  double kp, ki, kd;            // PID coefficients
  double previousError = 0.0;   // Previous error value
  double integral = 0.0;        // Integral of the error
  double setPoint = 0.0;        // Desired target value
};

#endif // PIDCONTROLLER_H