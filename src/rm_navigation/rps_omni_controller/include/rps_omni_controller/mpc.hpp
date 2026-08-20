#ifndef MPC_HPP_
#define MPC_HPP_

#include <Eigen/Dense>

#include <vector>

class OmniMpcController
{
public:
  using State = Eigen::Matrix<double, 4, 1>;
  using Control = Eigen::Vector2d;

  OmniMpcController(
    double Ts,
    int Np = 15,
    int Nc = 8,
    double a_min = -6.0,
    double a_max = 6.0,
    double v_min = -3.0,
    double v_max = 3.0);

  ~OmniMpcController() = default;

  void initWeights(
    const Eigen::Matrix4d & S,
    const Eigen::Matrix4d & Q,
    const Eigen::Matrix2d & R,
    const Eigen::Matrix2d & R_delta);

  void setHorizon(int Np, int Nc);

  void setAccelerationLimits(double a_min, double a_max);

  void setVelocityLimits(double v_min, double v_max);

  void setHoldLastControl(bool hold_last_control);

  Control solveAcceleration(
    const State & x0,
    const std::vector<State> & ref_seq);

  Eigen::Vector2d solveVelocityCommand(
    const State & x0,
    const std::vector<State> & ref_seq);

  void reset();

  const Eigen::Matrix4d & getA() const { return A_; }

  const Eigen::Matrix<double, 4, 2> & getB() const { return B_; }

  Control getLastAcceleration() const { return last_accel_; }

private:
  void updateModelMatrix();

  std::vector<State> normalizeReference(
    const std::vector<State> & ref_seq) const;

  void buildPredictionMatrix(
    Eigen::MatrixXd & A_bar,
    Eigen::MatrixXd & B_bar) const;

  void buildQPMatrix(
    const State & x0,
    const std::vector<State> & ref_seq,
    Eigen::MatrixXd & H,
    Eigen::VectorXd & f) const;

  Control clipAcceleration(const Control & u) const;

  Eigen::Vector2d clipVelocity(const Eigen::Vector2d & v) const;
  
  void buildConstraintMatrix(
    const State & x0,
    Eigen::MatrixXd & A_qp,
    Eigen::VectorXd & lower_bound,
    Eigen::VectorXd & upper_bound) const;

  bool solveConstrainedQP(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A_qp,
    const Eigen::VectorXd & lower_bound,
    const Eigen::VectorXd & upper_bound,
    Eigen::VectorXd & U) const;
private:
  double Ts_;

  int Np_;
  int Nc_;

  double a_min_;
  double a_max_;

  double v_min_;
  double v_max_;

  Eigen::Matrix4d A_;
  Eigen::Matrix<double, 4, 2> B_;

  Eigen::Matrix4d S_;  
  Eigen::Matrix4d Q_;  
  Eigen::Matrix2d R_;
  Eigen::Matrix2d R_delta_;

  Control last_accel_;

  Eigen::VectorXd last_solution_;

  bool hold_last_control_;

  double hessian_regularization_;
};

#endif
