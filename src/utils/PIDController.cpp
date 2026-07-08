#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd)
  : kp(kp), ki(ki), kd(kd)
{}

double PIDController::update(double currentValue) {
  // Calculate the error between the set point and the current value
  double error = setPoint - currentValue;
  
  // Update the integral of the error
  integral += error;

  // Calculate the derivative of the error
  double derivative = error - previousError;

  // Calculate the PID output
  double output = (kp * error) + (ki * integral) + (kd * derivative);
  
  // Update previous error for next iteration
  previousError = error;
  return output;
}