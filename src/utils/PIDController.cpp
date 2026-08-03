#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd)
  : _kp(kp), _ki(ki), _kd(kd)
{}

double PIDController::update(double currentValue) {
  // Calculate the error between the target point and the current value
  double error = _targetPoint - currentValue;

  // Update the integral of the error
  _integral += error;

  // Calculate the derivative of the error
  double derivative = error - _previousError;

  // Calculate the PID output
  double output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

  // Update previous error for next iteration
  _previousError = error;
  return output;
}

void PIDController::setPID(double kp, double ki, double kd) {
  setKp(kp);
  setKi(ki);
  setKd(kd);
}

void PIDController::setKp(double v) { setWithReset(_kp, v); }
void PIDController::setKi(double v) { setWithReset(_ki, v); }
void PIDController::setKd(double v) { setWithReset(_kd, v); }

void PIDController::setTargetPoint(double v) { setWithReset(_targetPoint, v); }

double PIDController::getKp() const { return _kp; }
double PIDController::getKi() const { return _ki; }
double PIDController::getKd() const { return _kd; }
double PIDController::getTargetPoint() const { return _targetPoint; }

void PIDController::reset() {
  _previousError = 0.0;
  _integral = 0.0;
}

void PIDController::setWithReset(double& member, double new_value) {
  // Only reset the controller state when the value actually changes
  if (member != new_value) {
    member = new_value;
    reset();
  }
}
