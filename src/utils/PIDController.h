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

  /* Set all PID coefficients at once (non-negative, resets state if changed)
   * `kp` - Proportional gain
   * `ki` - Integral gain
   * `kd` - Derivative gain
   */
  void setPID(double kp, double ki, double kd);

  // Set the proportional gain (resets state if changed)
  void setKp(double newKp);
  // Set the integral gain (resets state if changed)
  void setKi(double newKi);
  // Set the derivative gain (resets state if changed)
  void setKd(double newKd);

  /* Set the target value (resets state if changed) */
  void setTargetPoint(double newTargetPoint);

  /* Getters */
  double getKp() const;
  double getKi() const;
  double getKd() const;
  double getTargetPoint() const;

  /* Reset integral and previous error to zero */
  void reset();

private:
  // reset state only when the value actually changes
  void setWithReset(double& member, double new_value);

  double _kp = 0.0, _ki = 0.0, _kd  = 0.0;  // PID coefficients
  double _previousError = 0.0;  // Previous error value
  double _integral      = 0.0;  // Integral of the error
  double _targetPoint   = 0.0;  // Desired target value
};

#endif // PIDCONTROLLER_H
