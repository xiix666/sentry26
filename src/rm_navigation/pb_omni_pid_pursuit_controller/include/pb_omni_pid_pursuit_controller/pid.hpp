#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_

class PID
{
public:
  // kp -  proportional gain
  // ki -  Integral gain
  // kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  PID(double dt, double max, double min, double kp, double kd, double ki);

  // Returns the manipulated variable given a set_point and current process value
  double calculate(double set_point, double pv);
  void setSumError(double sum_error);
  void reset();
  void resetIntegral();
  void resetDerivative();

  ~PID();

private:
  double dt_;
  double max_;
  double min_;
  double kp_;
  double kd_;
  double ki_;
  double pre_error_;
  double integral_;
  bool has_pre_error_;
};

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
