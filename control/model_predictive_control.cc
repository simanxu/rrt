#include "control/model_predictive_control.h"

MPCController::MPCController(const CarModel& car, const MPCParams& params) {
  car_ = car;
  params_ = params;
  states_.setZero();
  control_.setZero();
  states_terminate_count_ = 0;
  logs_.open("../data/mpc_data.txt");
  if (!logs_.is_open()) {
    fprintf(stderr, "Failed open data file.\n");
  }
}

MPCController::~MPCController() {}

void MPCController::UpdateState(const CarStates& state, const CarStates& target) {
  states_prev_ = states_;
  states_ = state;
  target_ = target;
}

CarStates MPCController::NextState(const CarStates& state, const CarControl& control) {
  // k时刻状态
  double yaw_k = state(2);
  double v_k = state(3);
  double dt = car_.ct_dt;

  // 离散形式状态转移方程
  Eigen::MatrixXd A(n, n);
  A(0, 0) = 1.0;
  A(0, 1) = 0.0;
  A(0, 2) = -dt * v_k * std::sin(yaw_k);
  A(1, 0) = 0.0;
  A(1, 1) = 1.0;
  A(1, 2) = dt * v_k * std::cos(yaw_k);
  A(2, 0) = 0.0;
  A(2, 1) = 0.0;
  A(2, 2) = 1.0;
  Eigen::MatrixXd B(n, m);
  B(0, 0) = dt * std::cos(yaw_k);
  B(0, 1) = 0.0;
  B(1, 0) = dt * std::sin(yaw_k);
  B(1, 1) = 0.0;
  B(2, 0) = 0.0;
  B(2, 1) = dt;

  // x(k+1) - x(k) = A * 0 + B * u(k)
  CarStates state_next;
  state_next.head(n) = state.head(n) + B * control;
  double v_next = (control(0) + control(1)) * car_.radius / 2.0;
  double w_next = (control(1) - control(0)) * car_.radius / car_.width;
  state_next.tail(2) << v_next, w_next;
  return state_next;
}

CarControl MPCController::GetNextControl(double t) {
  // 定义问题维度和时间步长
  const int N = params_.horizon;  // 预测时间步数
  const double dt = car_.sp_dt;   // 预测时间步长
  double yaw_k = states_(2);
  double v_k = states_(3);

  // 离散形式状态转移方程
  Eigen::MatrixXd A(n, n);
  A(0, 0) = 1.0;
  A(0, 1) = 0.0;
  A(0, 2) = -dt * v_k * std::sin(yaw_k);
  A(1, 0) = 0.0;
  A(1, 1) = 1.0;
  A(1, 2) = dt * v_k * std::cos(yaw_k);
  A(2, 0) = 0.0;
  A(2, 1) = 0.0;
  A(2, 2) = 1.0;
  Eigen::MatrixXd B(n, m);
  B(0, 0) = dt * std::cos(yaw_k);
  B(0, 1) = 0.0;
  B(1, 0) = dt * std::sin(yaw_k);
  B(1, 1) = 0.0;
  B(2, 0) = 0.0;
  B(2, 1) = dt;

  // 定义迭代形式
  Eigen::MatrixXd Aqp = Eigen::MatrixXd::Zero(n * N, n * N);
  Eigen::MatrixXd Bqp = Eigen::MatrixXd::Zero(n * N, m * N);
  Eigen::Matrix3d A_pow[N + 1];
  A_pow[0].setIdentity();
  for (int i = 0; i < N; ++i) {
    A_pow[i + 1] = A * A_pow[i];
    Aqp.block(i * n, i * n, n, n) = A_pow[i + 1];
    for (int j = 0; j < i + 1; ++j) {
      Bqp.block(i * n, j * m, n, m) = A_pow[i - j] * B;
    }
  }

  // 定义目标状态
  Eigen::VectorXd x_now(n * N);
  Eigen::VectorXd x_ref(n * N);
  for (int i = 0; i < N; ++i) {
    x_now.segment(i * n, n) = states_.head(n);
    x_ref.segment(i * n, n) = target_.head(n);
  }

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n * N, n * N);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m * N, m * N);
  for (int i = 0; i < N; ++i) {
    Q.block(i * n, i * n, n, n) = params_.Q;
    R.block(i * m, i * m, m, m) = params_.R;
  }
  Q.bottomRightCorner(n, n) = params_.P;
  // std::cout << "Q\n" << Q << std::endl;
  // std::cout << "R\n" << R << std::endl;

  // 定义QP问题的约束和目标函数
  Eigen::MatrixXd H(m * N, m * N);
  Eigen::VectorXd g(m * N);
  Eigen::MatrixXd Aqp_t = Aqp.transpose();
  Eigen::MatrixXd Bqp_t = Bqp.transpose();
  H = R + Bqp_t * Q * Bqp;
  g = (x_now - x_ref).transpose() * Aqp_t * Q * Bqp;

  // 定义约束条件
  Eigen::VectorXd lbx = Eigen::VectorXd::Constant(n * N, -car_.v_max);
  Eigen::VectorXd ubx = Eigen::VectorXd::Constant(n * N, car_.v_max);

  // 构建QP问题
  qpOASES::QProblem qp(m * N, 0);
  qpOASES::Options options;
  options.setToDefault();
  options.terminationTolerance = 1e-6;
  options.printLevel = qpOASES::PL_NONE;
  options.enableRegularisation = qpOASES::BT_FALSE;
  qp.setOptions(options);

  // 解决QP问题
  int nWSR = 1000;
  qpOASES::returnValue qp_return = qp.init(H.data(), g.data(), 0, lbx.data(), ubx.data(), 0, 0, nWSR);
  if (qp_return != qpOASES::returnValue::SUCCESSFUL_RETURN) {
    fprintf(stdout, "Error init qp, error code %d.\n", qp_return);
  }

  Eigen::VectorXd control = Eigen::VectorXd::Zero(m * N);
  qp_return = qp.getPrimalSolution(control.data());
  if (qp_return != qpOASES::returnValue::SUCCESSFUL_RETURN) {
    fprintf(stdout, "Error solve qp, error code %d.\n", qp_return);
  } else {
    control_ = control.head(m);
  }

  // 输出控制输入并返回
  // std::cout << "control: " << control_.transpose() << std::endl;
  return control_;
}

bool MPCController::IsTerminate() {
  logs_ << states_.head(n).transpose() << std::endl;
  // if (((states_ - target_).cwiseAbs().array() < terninate_threshold).all()) {
  //   fprintf(stdout, "Targets termination condition reached.\n");
  // }

  if (((states_ - states_prev_).cwiseAbs().array() < states_threshold).all()) {
    ++states_terminate_count_;
    // fprintf(stdout, "States termination condition reached.\n");
  }
  bool is_terminate =
      ((states_ - target_).cwiseAbs().array() < terninate_threshold).all() || (states_terminate_count_ > 10);
  return is_terminate;
}
