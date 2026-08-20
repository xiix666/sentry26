#include "rps_omni_controller/pid.hpp"

PID::PID(double dt, double max, double min, double kp, double kd, double ki)
: dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki), pre_error_(0), integral_(0)
{
}

double PID::calculate(double set_point, double pv)
{
  double error = set_point - pv;

  double p_out = kp_ * error;

  integral_ += error * dt_;

  if (integral_ > 1.0) {
    integral_ = 1.0;
  } else if (integral_ < -1.0) {
    integral_ = -1.0;
  }

  double i_out = ki_ * integral_;

  double derivative = 0.0;

  if (has_pre_error_ && dt_ > 1e-9) {
    derivative = (error - pre_error_) / dt_;
  } else {
    derivative = 0.0;
    has_pre_error_ = true;
  }

  double d_out = kd_ * derivative;

  double output = p_out + i_out + d_out;

  if (output > max_) {
    output = max_;
  } else if (output < min_) {
    output = min_;
  }

  pre_error_ = error;

  return output;
}
void PID::reset()
{
  integral_ = 0.0;
  pre_error_ = 0.0;
  has_pre_error_ = false;
}

void PID::resetIntegral()
{
  integral_ = 0.0;
}

void PID::resetDerivative()
{
  pre_error_ = 0.0;
  has_pre_error_ = false;
}

void PID::setSumError(double sum_error)
{
  integral_ = sum_error;

  if (integral_ > 1.0) {
    integral_ = 1.0;
  } else if (integral_ < -1.0) {
    integral_ = -1.0;
  }
}

PID::~PID() {}
