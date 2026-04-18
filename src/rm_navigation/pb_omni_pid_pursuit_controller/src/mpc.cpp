// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0
#include "pb_omni_pid_pursuit_controller/mpc.hpp"

using namespace Eigen;

OmniMpcController::OmniMpcController(
  double Ts, int Np, int Nc,
   double v_min, double v_max)
: Ts_(Ts), Np_(Np), Nc_(Nc),
  v_min_(v_min), v_max_(v_max),
u_prev_(Vector2d::Zero())
{
  // 初始化MPC核心矩阵（超简化：2维纯平动）
  A_ = Matrix2d::Identity();  // 状态矩阵：I2
  B_ = Matrix2d::Identity() * Ts_; // 输入矩阵：Ts*I2

  // 初始化默认权重（工程推荐值，可通过initWeights覆盖）
  Q_ << 15.0, 0.0, 0.0, 15.0;    // 位置跟踪权重（越大跟踪越准）
  R_ << 0.5, 0.0, 0.0, 0.5;      // 输入幅值权重（越大速度越平缓）
  R_delta_ << 2.0, 0.0, 0.0, 2.0;// 输入增量权重（越大速度越平滑）
}
void OmniMpcController::setNp(double np){
  Np_ = np;
}
void OmniMpcController::setNc(double nc){
  Nc_ = nc;
}
void OmniMpcController::initWeights(const Matrix2d &Q, const Matrix2d &R, const Matrix2d &R_delta)
{
  Q_ = Q;
  R_ = R;
  R_delta_ = R_delta;
}

void OmniMpcController::setVelocityLimits(double v_min, double v_max)
{
  v_min_ = v_min;
  v_max_ = v_max;
}

// std::vector<Vector2d> OmniMpcController::generateRefSequence(
//   const Vector2d &curr_x,
//   const Vector2d &ref_x)
// {
//   std::vector<Vector2d> ref_seq(Np_);
//   // 线性插值生成参考序列（从当前位置到前瞻点，适配局部路径）
//   Vector2d step = (ref_x - curr_x) / (Np_ - 1);
//   for (int i = 0; i < Np_; ++i) {
//     ref_seq[i] = curr_x + step * i;
//   }
//   return ref_seq;
// }
void OmniMpcController::buildQPMatrix(
    const Eigen::Vector2d &curr_x,
    const std::vector<Eigen::Vector2d> &ref_seq,
    Eigen::MatrixXd &H,
    Eigen::VectorXd &f)
{
    int n_var = 2 * Nc_;

    H.setZero(n_var, n_var); 
    f.setZero(n_var);
    for (int i = 0; i < Np_; ++i) {
        int m = std::min(i, Nc_ - 1) + 1;
        Eigen::MatrixXd B_i_ext(2, n_var);
        B_i_ext.setZero();
        for (int j = 0; j < m; ++j) {
            B_i_ext.block<2, 2>(0, 2 * j) = B_;
        }

        Eigen::MatrixXd H_i = B_i_ext.transpose() * Q_ * B_i_ext;
        H += H_i;
        // std::cout << i << "H:" << H << std::endl;

        f += B_i_ext.transpose() * Q_ * (A_ * curr_x - ref_seq[i]);
    }
    
    // std::cout << "B:" << B_ << std::endl;
    // std::cout << "H:" << H << std::endl;
    // std::cout << "f:" << f<< std::endl;

    for (int i = 0; i < Nc_; ++i) {
        H.block<2, 2>(2 * i, 2 * i) += R_;
    }
    for (int j = 0; j < Nc_; ++j) {
      // 第j步控制量的索引：2*j ~ 2*j+1
      int idx_j = 2 * j;
      
      // （1）j=0：Δu_0 = u_0 - u_prev_（上一周期的控制量）
      if (j == 0) {
          // 增量惩罚对H的贡献：I^T * R_delta_ * I = R_delta_
          H.block<2, 2>(idx_j, idx_j) += R_delta_;
          // 增量惩罚对f的贡献：I^T * R_delta_ * (-u_prev_)
          f.segment<2>(idx_j) += R_delta_ * (-u_prev_);
      } 
      // （2）j>0：Δu_j = u_j - u_{j-1}
      else {
          int idx_j_1 = 2 * (j - 1); // 上一步控制量的索引
          // 对H的贡献：(I, -I)^T * R_delta_ * (I, -I)
          H.block<2, 2>(idx_j, idx_j) += R_delta_;          // u_j^T * R_delta_ * u_j
          H.block<2, 2>(idx_j_1, idx_j_1) += R_delta_;      // u_{j-1}^T * R_delta_ * u_{j-1}
          H.block<2, 2>(idx_j, idx_j_1) -= R_delta_;         // -u_j^T * R_delta_ * u_{j-1}
          H.block<2, 2>(idx_j_1, idx_j) -= R_delta_;         // -u_{j-1}^T * R_delta_ * u_j
          // 对f的贡献：0（因为Δu_j无常数项）
      }
  }
}

Vector2d OmniMpcController::clipVelocity(const Eigen::Vector2d &u)
{
  // 速度裁剪：将v_x/v_y限制在[v_min_, v_max_]范围内
  Vector2d u_clipped;
  u_clipped(0) = std::clamp(u(0), v_min_, v_max_);
  u_clipped(1) = std::clamp(u(1), v_min_, v_max_);
  return u_clipped;
}

Vector2d OmniMpcController::solve(
    const Eigen::Vector2d &curr_x,
    const std::vector<Eigen::Vector2d> &ref_seq)
  {
  
    // 2. 构建QP问题的H（海森矩阵）和f（梯度向量）
    int n_var = 2 * Nc_;
    MatrixXd H(n_var, n_var);
    VectorXd f(n_var);
    buildQPMatrix(curr_x, ref_seq, H, f);
    // std::cout << H.transpose() << std::endl;
    // 3. Eigen解析求解无约束QP：min J = 0.5*U^T*H*U + f^T*U
    // 解析解：U* = -H^{-1}*f （利用Cholesky分解求逆，数值稳定）
    VectorXd u_opt_seq(n_var);
    try {
      // Cholesky分解（LLT）：仅适用于对称正定矩阵（MPC的H满足该条件）
      u_opt_seq = -H.llt().solve(f);
    } catch (const std::exception &e) {
      // 异常处理（如矩阵奇异）：返回零速度
      RCLCPP_WARN(rclcpp::get_logger("OmniMpcController"), "QP solve failed: %s, return zero velocity", e.what());
      return Vector2d::Zero();
    }
    std::cout << "u_opt_seq: " << u_opt_seq.transpose() << std::endl;
    // 4. 提取第一个控制量（滚动时域原则）
    Vector2d u_curr = u_opt_seq.head<2>();
  
    // 5. 速度裁剪：替代原硬约束，满足工程精度
    u_curr = clipVelocity(u_curr);
  
    // 6. 保存当前输入，用于下一周期增量计算
    u_prev_ = u_curr;
  
    return u_curr;
  }

void OmniMpcController::reset()
{
  u_prev_.setZero();    // 重置历史输入

}

