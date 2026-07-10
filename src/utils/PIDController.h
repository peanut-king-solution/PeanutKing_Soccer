#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
  // Constructor to initialize PID coefficients
  PIDController(double kp, double ki, double kd);

  /* Update the PID controller with the current value and return the control output.
   * `currentValue` - The current value of the process variable

   * `Returns` - The control output based on the PID calculation
   */
  double update(double currentValue);

// variables
  double kp, ki, kd;            // PID coefficients
  double previousError = 0.0;   // Previous error value
  double integral = 0.0;        // Integral of the error
  double setPoint = 0.0;        // Desired target value
};

#endif // PIDCONTROLLER_H