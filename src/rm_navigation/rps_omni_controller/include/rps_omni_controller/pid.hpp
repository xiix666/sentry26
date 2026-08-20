#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_

class PID
{
public:

  PID(double dt, double max, double min, double kp, double kd, double ki);

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

#endif
