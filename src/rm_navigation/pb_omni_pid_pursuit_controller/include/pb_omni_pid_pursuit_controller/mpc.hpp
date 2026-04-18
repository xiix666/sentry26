#ifndef MCP_HPP_
#define MPCHPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <ceres/ceres.h>
class OmniMpcController
{
public:
  /**
   * @brief 构造函数
   * @param Ts 控制周期（离散步长，s）
   * @param Np 预测时域
   * @param Nc 控制时域
   * @param omega_fixed 定值角速度（rad/s）
   * @param v_min 线速度最小值（m/s）
   * @param v_max 线速度最大值（m/s）
   */
  OmniMpcController(
    double Ts, int Np = 6, int Nc = 3,
     double v_min = -3.0, double v_max = 3.0);

  /**
   * @brief 析构函数
   */
  ~OmniMpcController() = default;

  /**
   * @brief 初始化MPC权重矩阵（核心调参项）
   * @param Q 位置跟踪权重（2x2，对角正定）
   * @param R 输入幅值权重（2x2，对角正定）
   * @param R_delta 输入增量权重（2x2，对角正定）
   */
  void initWeights(const Eigen::Matrix2d &Q, const Eigen::Matrix2d &R, const Eigen::Matrix2d &R_delta);

  /**
   * @brief 设置速度约束（支持动态更新，如曲率/逼近速度缩放）
   * @param v_min 新的线速度最小值（m/s）
   * @param v_max 新的线速度最大值（m/s）
   */
  void setVelocityLimits(double v_min, double v_max);

  /**
   * @brief 求解MPC最优控制量
   * @param curr_x 机器人当前位置（本体坐标系，2维：x, y）
   * @param ref_x 参考目标位置（本体坐标系，2维：x, y，前瞻点）
   * @return 最优控制输入（2维：v_x, v_y）
   */
  Eigen::Vector2d solve(
    const Eigen::Vector2d &curr_x,
    const std::vector<Eigen::Vector2d> &ref_seq);

  /**
   * @brief 获取定值角速度
   * @return 定值角速度（rad/s）
   */
  double getFixedOmega() const { return omega_fixed_; }

  /**
   * @brief 重置MPC控制器（如碰撞后、路径重置时）
   */
  void reset();
  void setNp(double np);
  void setNc(double nc);
private:
  // MPC核心参数
  double Ts_;          // 控制周期（s）
  int Np_;             // 预测时域
  int Nc_;             // 控制时域
  double omega_fixed_; // 定值角速度（rad/s）
  double v_min_;       // 线速度最小值
  double v_max_;       // 线速度最大值

  // MPC矩阵（2维状态+2维输入，超简化）
  Eigen::Matrix2d A_;  // 状态矩阵 [I2]
  Eigen::Matrix2d B_;  // 输入矩阵 [Ts*I2]

  // 权重矩阵
  Eigen::Matrix2d Q_;  // 位置跟踪权重
  Eigen::Matrix2d R_;  // 输入幅值权重
  Eigen::Matrix2d R_delta_; // 输入增量权重

  // 历史输入（用于计算输入增量，抑制速度突变）
  Eigen::Vector2d u_prev_;

  /**
   * @brief 生成预测时域内的参考位置序列
   * @param curr_x 当前位置
   * @param ref_x 目标参考位置
   * @return 参考位置序列（Np个2维向量）
   */
  std::vector<Eigen::Vector2d> generateRefSequence(
    const Eigen::Vector2d &curr_x,
    const Eigen::Vector2d &ref_x);

  /**
   * @brief 构建QP问题的海森矩阵H和梯度向量f
   * @param curr_x 当前位置
   * @param ref_seq 参考位置序列
   * @param H 输出海森矩阵（n_var_ x n_var_）
   * @param f 输出梯度向量（n_var_ x 1）
   */
  void buildQPMatrix(
    const Eigen::Vector2d &curr_x,
    const std::vector<Eigen::Vector2d> &ref_seq,
    Eigen::MatrixXd &H,
    Eigen::VectorXd &f);

  /**
   * @brief 构建QP问题的约束（上下界）
   * @param lb 输出变量下界（n_var_ x 1）
   * @param ub 输出变量上界（n_var_ x 1）
   */
  void buildQPCons(Eigen::VectorXd &lb, Eigen::VectorXd &ub);
  Eigen::Vector2d clipVelocity(const Eigen::Vector2d &u);
};

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER_OMNI_MPC_CONTROLLER_HPP_