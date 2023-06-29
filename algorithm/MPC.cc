#include <iostream>
#include <qpOASES.hpp>
#include <vector>

// 二维点结构体
struct Point2D {
  double x;
  double y;
};

// 两轮移动底盘结构体
struct MobileRobot {
  double wheelbase;  // 轮距
  double max_speed;  // 最大速度
  double max_accel;  // 最大加速度
};

// MPC跟踪器结构体
struct MPC {
  MobileRobot robot;                      // 两轮移动底盘
  std::vector<Point2D> reference_points;  // 参考点
  double dt;                              // 时间步长
  int horizon;                            // 预测时间步数
};

// 计算底盘状态矩阵
Eigen::MatrixXd calculateStateMatrix(MPC mpc, int i) {
  Eigen::MatrixXd A(3, 3);
  A << 1, 0, -mpc.robot.max_speed * mpc.dt * sin(i * mpc.dt * mpc.robot.max_speed / mpc.robot.wheelbase), 0, 1,
      mpc.robot.max_speed * mpc.dt * cos(i * mpc.dt * mpc.robot.max_speed / mpc.robot.wheelbase), 0, 0, 1;
  return A;
}

// 计算底盘输入矩阵
Eigen::MatrixXd calculateInputMatrix(MPC mpc, int i) {
  Eigen::MatrixXd B(3, 2);
  B << mpc.dt * cos(i * mpc.dt * mpc.robot.max_speed / mpc.robot.wheelbase), 0,
      mpc.dt * sin(i * mpc.dt * mpc.robot.max_speed / mpc.robot.wheelbase), 0, 0, mpc.dt;
  return B;
}

// 计算底盘状态约束矩阵
Eigen::MatrixXd calculateStateConstraintMatrix(MPC mpc) {
  Eigen::MatrixXd A(2 * (mpc.horizon + 1), 3 * (mpc.horizon + 1));
  for (int i = 0; i < mpc.horizon + 1; i++) {
    A.block<2, 3>(2 * i, 3 * i) = Eigen::MatrixXd::Identity(2, 3);
  }
  return A;
}

// 计算底盘状态约束边界
Eigen::VectorXd calculateStateConstraintBoundary(MPC mpc, Point2D current_state) {
  Eigen::VectorXd b(2 * (mpc.horizon + 1));
  for (int i = 0; i < mpc.horizon + 1; i++) {
    b.segment<2>(2 * i) << current_state.x, current_state.y;
  }
  return b;
}

// 计算底盘输入约束矩阵
Eigen::MatrixXd calculateInputConstraintMatrix(MPC mpc) {
  Eigen::MatrixXd A(2 * mpc.horizon, 2 * (mpc.horizon + 1));
  for (int i = 0; i < mpc.horizon; i++) {
    A.block<2, 2>(2 * i, 2 * i) = Eigen::MatrixXd::Identity(2, 2);
    A.block<2, 2>(2 * i, 2 * (i + 1)) = -Eigen::MatrixXd::Identity(2, 2);
  }
  return A;
}

// 计算底盘输入约束边界
Eigen::VectorXd calculateInputConstraintBoundary(MPC mpc) {
  Eigen::VectorXd b(2 * mpc.horizon);
  b << mpc.robot.max_accel, mpc.robot.max_accel, -mpc.robot.max_accel, -mpc.robot.max_accel, mpc.robot.max_accel,
      mpc.robot.max_accel, -mpc.robot.max_accel, -mpc.robot.max_accel, Eigen::VectorXd::Zero(2 * (mpc.horizon - 2));
  return b;
}

// 计算代价函数矩阵
Eigen::MatrixXd calculateCostMatrix(MPC mpc) {
  Eigen::MatrixXd Q(3, 3);
  Q << 1, 0, 0, 0, 1, 0, 0, 0, 0.1;
  Eigen::MatrixXd R(2, 2);
  R << 0.1, 0, 0, 0.1;
  Eigen::MatrixXd H =
      Eigen::MatrixXd::Zero(3 * (mpc.horizon + 1) + 2 * mpc.horizon, 3 * (mpc.horizon + 1) + 2 * mpc.horizon);
  for (int i = 0; i < mpc.horizon; i++) {
    H.block<3, 3>(3 * i, 3 * i) += Q;
    H.block<2, 2>(3 * (mpc.horizon + i) + 2 * i, 3 * (mpc.horizon + i) + 2 * i) += R;
  }
  H.block<3, 3>(3 * (mpc.horizon), 3 * (mpc.horizon)) += Q;
  return H;
}

// 计算代价函数边界
Eigen::VectorXd calculateCostBoundary(MPC mpc, Point2D current_state) {
  Eigen::VectorXd f(3 * (mpc.horizon + 1) + 2 * mpc.horizon);
  for (int i = 0; i < mpc.horizon; i++) {
    f.segment<3>(3 * i) << -mpc.reference_points[i].x, -mpc.reference_points[i].y, -current_state.x;
    f.segment<2>(3 * (mpc.horizon + i) + 2 * i) << -mpc.reference_points[i].x, -mpc.reference_points[i].y;
  }
  f.segment<3>(3 * (mpc.horizon)) << -mpc.reference_points[mpc.horizon].x, -mpc.reference_points[mpc.horizon].y,
      -current_state.x;
  return f;
}

// MPC跟踪函数
Point2D trackMPC(MPC mpc, Point2D current_state) {
  // 初始化qpOASES求解器
  qpOASES::Options options;
  qpOASES::SQProblem solver(3 * (mpc.horizon + 1) + 2 * mpc.horizon, 2 * (mpc.horizon + 1));

  // 计算qpOASES所需的矩阵和向量
  Eigen::MatrixXd H = calculateCostMatrix(mpc);
  Eigen::VectorXd f = calculateCostBoundary(mpc, current_state);
  Eigen::MatrixXd Aeq = calculateStateConstraintMatrix(mpc);
  Eigen::VectorXd beq = calculateStateConstraintBoundary(mpc, current_state);
  Eigen::MatrixXd Ain = calculateInputConstraintMatrix(mpc);
  Eigen::VectorXd bin = calculateInputConstraintBoundary(mpc);

  // 求解QP问题
  qpOASES::int_t nWSR = options.initialization.maxIter;
  qpOASES::returnValue status =
      solver.init(H.data(), f.data(), Ain.data(), nullptr, Aeq.data(), beq.data(), bin.data(), nWSR);

  if (status != qpOASES::SUCCESSFUL_RETURN) {
    std::cerr << "qpOASES solver failed to initialize!" << std::endl;
    return current_state;
  }

  qpOASES::real_t xOpt[3 * (mpc.horizon + 1) + 2 * mpc.horizon];

  status = solver.getPrimalSolution(xOpt);

  if (status != qpOASES::SUCCESSFUL_RETURN) {
    std::cerr << "qpOASES solver failed to solve the problem!" << std::endl;
    return current_state;
  }

  // 获取下一时刻底盘状态
  Point2D next_state = {xOpt[3], xOpt[6]};

  return next_state;
}

int main() {
  // 创建MPC跟踪器
  MPC mpc;
  mpc.robot.wheelbase = 1.0;
  mpc.robot.max_speed = 2.0;
  mpc.robot.max_accel = 1.0;
  mpc.dt = 0.1;
  mpc.horizon = 10;

  // 添加参考点
  std::vector<Point2D> reference_points = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 1.0}, {4.0, 0.0}};
  mpc.reference_points = reference_points;

  // 计算底盘轨迹
  std::vector<Point2D> trajectory = calculateTrajectory(mpc);

  // 输出底盘轨迹
  std::cout << "底盘轨迹：" << std::endl;
  for (auto point : trajectory) {
    std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
  }

  return 0;
}
