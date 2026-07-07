#ifndef MPC_HPP_
#define MPC_HPP_

#include <Eigen/Dense>

#include <vector>

class OmniMpcController
{
public:
  using State = Eigen::Matrix<double, 4, 1>;   // [px, py, vx, vy]^T
  using Control = Eigen::Vector2d;             // [ax, ay]^T

  /**
   * @brief 二阶积分模型 MPC 控制器
   *
   * 状态量:
   *   x = [px, py, vx, vy]^T
   *
   * 控制量:
   *   u = [ax, ay]^T
   *
   * 离散模型:
   *   x(k+1) = A x(k) + B u(k)
   *
   * @param Ts 控制周期，单位 s
   * @param Np 预测时域
   * @param Nc 控制时域
   * @param a_min 加速度最小值
   * @param a_max 加速度最大值
   * @param v_min 速度最小值
   * @param v_max 速度最大值
   */
  OmniMpcController(
    double Ts,
    int Np = 15,
    int Nc = 8,
    double a_min = -6.0,
    double a_max = 6.0,
    double v_min = -3.0,
    double v_max = 3.0);

  ~OmniMpcController() = default;

  /**
   * @brief 初始化权重矩阵
   *
   * Q:
   *   状态误差权重，4x4，对应 [px, py, vx, vy]
   *
   * R:
   *   控制输入权重，2x2，对应 [ax, ay]
   *
   * R_delta:
   *   控制输入变化率权重，2x2，对应 Δa
   *   如果想完全按标准形式 J = track + U^T R U，
   *   可以把 R_delta 设为 0 矩阵。
   */
  void initWeights(
    const Eigen::Matrix4d & S,
    const Eigen::Matrix4d & Q,
    const Eigen::Matrix2d & R,
    const Eigen::Matrix2d & R_delta);

  void setHorizon(int Np, int Nc);

  void setAccelerationLimits(double a_min, double a_max);

  void setVelocityLimits(double v_min, double v_max);

  void setHoldLastControl(bool hold_last_control);

  /**
   * @brief 求解当前最优加速度
   *
   * @param x0 当前状态 [px, py, vx, vy]^T
   * @param ref_seq 参考状态序列，长度最好 >= Np
   * @return 当前周期使用的加速度 [ax, ay]^T
   */
  Control solveAcceleration(
    const State & x0,
    const std::vector<State> & ref_seq);

  /**
   * @brief 直接求解速度命令
   *
   * 内部先求解加速度:
   *   a_cmd = solveAcceleration(...)
   *
   * 然后积分:
   *   v_cmd = v_now + Ts * a_cmd
   *
   * @param x0 当前状态 [px, py, vx, vy]^T
   * @param ref_seq 参考状态序列
   * @return 速度命令 [vx, vy]^T
   */
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

#endif  // MPC_HPP_