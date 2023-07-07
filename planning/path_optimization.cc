#include "planning/path_optimization.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <qpOASES.hpp>

namespace {
const double kTiny = 1e-6;
}  // namespace

PathOptimization::PathOptimization(int order) { order_ = order + 1; }

PathOptimization::~PathOptimization() {}

/**
 * @brief 5阶曲线，p、v、a连续约束
 * @note 3: a*t^3 + b*t^2 + c*t + d
 * @note 5: a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f
 * @param path: 输入的轨迹
 */
void PathOptimization::OptimizePath(const std::vector<PathData>& path) {
  len_path_ = path.size();
  const int num_vars = order_ * (len_path_ - 1);
  const int num_csts = 4 * len_path_;

  time_stamp_.resize(len_path_);
  for (int i = 0; i < len_path_; ++i) {
    time_stamp_[i] = path[i].time;
    std::cout << "time_stamp: " << i << " " << path[i].time << std::endl;
  }

  using MatrixRowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  MatrixRowMajor Hess = MatrixRowMajor::Zero(num_vars, num_vars);
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(num_vars);
  for (int i = 0; i < len_path_ - 1; ++i) {
    double t = path[i].time;
    double t_2 = t * t;
    double t_3 = t_2 * t;
    int nv = i * order_;
    if (order_ == 4) {
      Hess(nv, nv) = 6 * t * 6 * t;
      Hess(nv + 1, nv + 1) = 2 * 2;
    } else if (order_ == 6) {
      Hess(nv, nv) = 20 * t_3 * 20 * t_3;
      Hess(nv + 1, nv + 1) = 12 * t_2 * 12 * t_2;
      Hess(nv + 2, nv + 2) = 6 * t * 6 * t;
      Hess(nv + 3, nv + 3) = 2 * 2;
    }
  }
  //   std::cout << "Hess = [\n" << Hess << "];" << std::endl;
  //   std::cout << "grad = [" << grad.transpose() << "];" << std::endl;

  MatrixRowMajor Cst = MatrixRowMajor::Zero(num_csts, num_vars);
  Eigen::VectorXd lbc = Eigen::VectorXd::Constant(num_csts, -kTiny);
  Eigen::VectorXd ubc = Eigen::VectorXd::Constant(num_csts, kTiny);
  for (int ncol = 0; ncol < 3; ++ncol) {
    curve_[ncol].setZero(num_vars);
    for (int i = 0; i < len_path_; ++i) {
      double t = path[i].time;
      double t_2 = t * t;
      double t_3 = t_2 * t;
      double t_4 = t_3 * t;
      double t_5 = t_4 * t;
      int nc = i * 4;
      int nv = i * order_;
      if (order_ == 4) {
        if (i == 0) {
          // p_{start} = p0
          Cst(nc, nv) = t_3;
          Cst(nc, nv + 1) = t_2;
          Cst(nc, nv + 2) = t;
          Cst(nc, nv + 3) = 1;
          lbc(nc) = path[i].pos(ncol) - kTiny;
          ubc(nc) = path[i].pos(ncol) + kTiny;
          // v_{start} = 0
          Cst(nc + 1, nv) = 3 * t_2;
          Cst(nc + 1, nv + 1) = 2 * t;
          Cst(nc + 1, nv + 2) = 1;
          lbc(nc + 1) = -kTiny;
          ubc(nc + 1) = kTiny;
          // a_{start} = 0
          Cst(nc + 2, nv) = 6 * t;
          Cst(nc + 2, nv + 1) = 2;
          lbc(nc + 2) = -kTiny;
          ubc(nc + 2) = kTiny;
        } else if (i == len_path_ - 1) {
          // p_{end} = pt
          Cst(nc, nv - 4) = t_3;
          Cst(nc, nv - 3) = t_2;
          Cst(nc, nv - 2) = t;
          Cst(nc, nv - 1) = 1;
          lbc(nc) = path[i].pos(ncol) - kTiny;
          ubc(nc) = path[i].pos(ncol) + kTiny;
          // v_{end} = 0
          Cst(nc + 1, nv - 4) = 3 * t_2;
          Cst(nc + 1, nv - 3) = 2 * t;
          Cst(nc + 1, nv - 2) = 1;
          lbc(nc + 1) = -kTiny;
          ubc(nc + 1) = kTiny;
          // a_{end} = 0
          Cst(nc + 2, nv - 4) = 6 * t;
          Cst(nc + 2, nv - 3) = 2;
          lbc(nc + 2) = -kTiny;
          ubc(nc + 2) = kTiny;
        } else {
          // p_i = path[i]
          Cst(nc, nv) = t_3;
          Cst(nc, nv + 1) = t_2;
          Cst(nc, nv + 2) = t;
          Cst(nc, nv + 3) = 1;
          lbc(nc) = path[i].pos(ncol) - kTiny;
          ubc(nc) = path[i].pos(ncol) + kTiny;
          // p_i = p_{i-1}
          Cst(nc + 1, nv - 4) = -t_3;
          Cst(nc + 1, nv - 3) = -t_2;
          Cst(nc + 1, nv - 2) = -t;
          Cst(nc + 1, nv - 1) = -1;
          Cst(nc + 1, nv) = t_3;
          Cst(nc + 1, nv + 1) = t_2;
          Cst(nc + 1, nv + 2) = t;
          Cst(nc + 1, nv + 3) = 1;
          lbc(nc + 1) = -kTiny;
          ubc(nc + 1) = kTiny;
          // v_i = v_{i-1}
          Cst(nc + 2, nv - 4) = -3 * t_2;
          Cst(nc + 2, nv - 3) = -2 * t;
          Cst(nc + 2, nv - 2) = -1;
          Cst(nc + 2, nv) = 3 * t_2;
          Cst(nc + 2, nv + 1) = 2 * t;
          Cst(nc + 2, nv + 2) = 1;
          lbc(nc + 2) = -kTiny;
          ubc(nc + 2) = kTiny;
          // a_i = a_{i-1}
          Cst(nc + 3, nv - 4) = -6 * t;
          Cst(nc + 3, nv - 3) = -2;
          Cst(nc + 3, nv) = 6 * t;
          Cst(nc + 3, nv + 1) = 2;
          lbc(nc + 3) = -kTiny;
          ubc(nc + 3) = kTiny;
        }
      } else if (order_ == 6) {
        if (i == 0) {
          // p_{start} = p0
          Cst(nc, nv) = t_5;
          Cst(nc, nv + 1) = t_4;
          Cst(nc, nv + 2) = t_3;
          Cst(nc, nv + 3) = t_2;
          Cst(nc, nv + 4) = t;
          Cst(nc, nv + 5) = 1;
          lbc(nc) = path[i].pos(ncol) - kTiny;
          ubc(nc) = path[i].pos(ncol) + kTiny;
          // v_{start} = 0
          Cst(nc + 1, nv) = 5 * t_4;
          Cst(nc + 1, nv + 1) = 4 * t_3;
          Cst(nc + 1, nv + 2) = 3 * t_2;
          Cst(nc + 1, nv + 3) = 2 * t;
          Cst(nc + 1, nv + 4) = 1;
          lbc(nc + 1) = -kTiny;
          ubc(nc + 1) = kTiny;
          // a_{start} = 0
          Cst(nc + 2, nv) = 20 * t_3;
          Cst(nc + 2, nv + 1) = 12 * t_2;
          Cst(nc + 2, nv + 2) = 6 * t;
          Cst(nc + 2, nv + 3) = 2;
          lbc(nc + 2) = -kTiny;
          ubc(nc + 2) = kTiny;
        } else if (i == len_path_ - 1) {
          // p_{end} = pt
          Cst(nc, nv - 6) = t_5;
          Cst(nc, nv - 5) = t_4;
          Cst(nc, nv - 4) = t_3;
          Cst(nc, nv - 3) = t_2;
          Cst(nc, nv - 2) = t;
          Cst(nc, nv - 1) = 1;
          lbc(nc) = path[i].pos(ncol) - kTiny;
          ubc(nc) = path[i].pos(ncol) + kTiny;
          // v_{end} = 0
          Cst(nc + 1, nv - 6) = 5 * t_4;
          Cst(nc + 1, nv - 5) = 4 * t_3;
          Cst(nc + 1, nv - 4) = 3 * t_2;
          Cst(nc + 1, nv - 3) = 2 * t;
          Cst(nc + 1, nv - 2) = 1;
          lbc(nc + 1) = -kTiny;
          ubc(nc + 1) = kTiny;
          // a_{end} = 0
          Cst(nc + 2, nv - 6) = 20 * t_3;
          Cst(nc + 2, nv - 5) = 12 * t_2;
          Cst(nc + 2, nv - 4) = 6 * t;
          Cst(nc + 2, nv - 3) = 2;
          lbc(nc + 2) = -kTiny;
          ubc(nc + 2) = kTiny;
        } else {
          // p_i = path[i]
          Cst(nc, nv) = t_5;
          Cst(nc, nv + 1) = t_4;
          Cst(nc, nv + 2) = t_3;
          Cst(nc, nv + 3) = t_2;
          Cst(nc, nv + 4) = t;
          Cst(nc, nv + 5) = 1;
          lbc(nc) = path[i].pos(ncol) - kTiny;
          ubc(nc) = path[i].pos(ncol) + kTiny;
          // p_i = p_{i-1}
          Cst(nc + 1, nv - 6) = -t_5;
          Cst(nc + 1, nv - 5) = -t_4;
          Cst(nc + 1, nv - 4) = -t_3;
          Cst(nc + 1, nv - 3) = -t_2;
          Cst(nc + 1, nv - 2) = -t;
          Cst(nc + 1, nv - 1) = -1;
          Cst(nc + 1, nv) = t_5;
          Cst(nc + 1, nv + 1) = t_4;
          Cst(nc + 1, nv + 2) = t_3;
          Cst(nc + 1, nv + 3) = t_2;
          Cst(nc + 1, nv + 4) = t;
          Cst(nc + 1, nv + 5) = 1;
          lbc(nc + 1) = -kTiny;
          ubc(nc + 1) = kTiny;
          // v_i = v_{i-1}
          Cst(nc + 2, nv - 6) = -5 * t_4;
          Cst(nc + 2, nv - 5) = -4 * t_3;
          Cst(nc + 2, nv - 4) = -3 * t_2;
          Cst(nc + 2, nv - 3) = -2 * t;
          Cst(nc + 2, nv - 2) = -1;
          Cst(nc + 2, nv) = 5 * t_4;
          Cst(nc + 2, nv + 1) = 4 * t_3;
          Cst(nc + 2, nv + 2) = 3 * t_2;
          Cst(nc + 2, nv + 3) = 2 * t;
          Cst(nc + 2, nv + 4) = 1;
          lbc(nc + 2) = -kTiny;
          ubc(nc + 2) = kTiny;
          // a_i = a_{i-1}
          Cst(nc + 3, nv - 6) = -20 * t_3;
          Cst(nc + 3, nv - 5) = -12 * t_2;
          Cst(nc + 3, nv - 4) = -6 * t;
          Cst(nc + 3, nv - 3) = -2;
          Cst(nc + 3, nv) = 20 * t_3;
          Cst(nc + 3, nv + 1) = 12 * t_2;
          Cst(nc + 3, nv + 2) = 6 * t;
          Cst(nc + 3, nv + 3) = 2;
          lbc(nc + 3) = -kTiny;
          ubc(nc + 3) = kTiny;
        }
      }
    }
    // std::cout << "Cst = [\n" << Cst << "];" << std::endl;
    // std::cout << "lbc = [" << lbc.transpose() << "];" << std::endl;
    // std::cout << "ubc = [" << ubc.transpose() << "];" << std::endl;

    qpOASES::QProblem qp(num_vars, num_csts);
    qpOASES::Options options;
    options.setToDefault();
    options.terminationTolerance = 1e-6;
    options.printLevel = qpOASES::PL_NONE;
    qp.setOptions(options);
    int nWSR = 1e6;
    qpOASES::returnValue qp_return = qp.init(Hess.data(), grad.data(), Cst.data(), 0, 0, lbc.data(), ubc.data(), nWSR);
    if (qp_return != qpOASES::SUCCESSFUL_RETURN) {
      fprintf(stdout, "Error init qp, error code %d.\n", qp_return);
    } else {
    }

    qp_return = qp.getPrimalSolution(curve_[ncol].data());
    if (qp_return != qpOASES::returnValue::SUCCESSFUL_RETURN) {
      fprintf(stdout, "Error solve qp, error code %d.\n", qp_return);
    }

    // std::cout << ncol << " dim trajectory curve solved in nWSR = " << nWSR << std::endl;
    // std::cout << "opti_value = [" << curve_[ncol].transpose() << "];" << std::endl;
    // std::cout << "Cst * Opt = " << (Cst * curve_[ncol]).transpose() << std::endl;
    // std::cout << "Hess size = " << Hess.rows() << ", " << Hess.cols() << std::endl;
    // std::cout << "grad size = " << grad.rows() << ", " << grad.cols() << std::endl;
    // std::cout << "Cst  size = " << Cst.rows() << ", " << Cst.cols() << std::endl;
    // std::cout << "lbc  size = " << lbc.rows() << ", " << lbc.cols() << std::endl;
    // std::cout << "ubc  size = " << ubc.rows() << ", " << ubc.cols() << std::endl;
  }
}

void PathOptimization::SaveOptimizedPath(const std::vector<PathData>& path) {
  std::ofstream recoder;
  recoder.open("../data/opt_path.txt");
  if (!recoder.is_open()) {
    printf("Failed to open opt_path.txt file!\n");
    return;
  }

  for (int i = 0; i < path.size() - 1; ++i) {
    for (double t = path[i].time; t <= path[i + 1].time; t += 0.01) {
      int n = i * order_;
      double t_2 = t * t;
      double t_3 = t_2 * t;
      double t_4 = t_3 * t;
      double t_5 = t_4 * t;
      double x, y, z, vx, vy, vz;
      if (order_ == 4) {
        x = curve_[0](n) * t_3 + curve_[0](n + 1) * t_2 + curve_[0](n + 2) * t + curve_[0](n + 3);
        y = curve_[1](n) * t_3 + curve_[1](n + 1) * t_2 + curve_[1](n + 2) * t + curve_[1](n + 3);
        z = curve_[2](n) * t_3 + curve_[2](n + 1) * t_2 + curve_[2](n + 2) * t + curve_[2](n + 3);
        vx = 3 * curve_[0](n) * t_2 + 2 * curve_[0](n + 1) * t + curve_[0](i * 3 + 2);
        vy = 3 * curve_[1](n) * t_2 + 2 * curve_[1](n + 1) * t + curve_[1](i * 3 + 2);
        vz = 3 * curve_[2](n) * t_2 + 2 * curve_[2](n + 1) * t + curve_[2](i * 3 + 2);
      } else if (order_ == 6) {
        x = curve_[0](n) * t_5 + curve_[0](n + 1) * t_4 + curve_[0](n + 2) * t_3 + curve_[0](n + 3) * t_2 +
            curve_[0](n + 4) * t + curve_[0](n + 5);
        y = curve_[1](n) * t_5 + curve_[1](n + 1) * t_4 + curve_[1](n + 2) * t_3 + curve_[1](n + 3) * t_2 +
            curve_[1](n + 4) * t + curve_[1](n + 5);
        z = curve_[2](n) * t_5 + curve_[2](n + 1) * t_4 + curve_[2](n + 2) * t_3 + curve_[2](n + 3) * t_2 +
            curve_[2](n + 4) * t + curve_[2](n + 5);
        vx = 5 * curve_[0](n) * t_4 + 4 * curve_[0](n + 1) * t_3 + 3 * curve_[0](n + 2) * t_2 +
             2 * curve_[0](n + 3) * t + curve_[0](n + 4);
        vy = 5 * curve_[1](n) * t_4 + 4 * curve_[1](n + 1) * t_3 + 3 * curve_[1](n + 2) * t_2 +
             2 * curve_[1](n + 3) * t + curve_[1](n + 4);
        vz = 5 * curve_[2](n) * t_4 + 4 * curve_[2](n + 1) * t_3 + 3 * curve_[2](n + 2) * t_2 +
             2 * curve_[2](n + 3) * t + curve_[2](n + 4);
      }
      recoder << t << " " << x << " " << y << " " << z << " " << vx << " " << vy << " " << vz << std::endl;
      //   fprintf(stdout, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f\n", t, x, y, z, vx, vy, vz);
    }
  }
  recoder.close();
}

PathData PathOptimization::GetOptimizedPath(double t) {
  t = std::clamp(t, 0.0, time_stamp_.tail(1)(0));
  int index = BinarySearch(t);
  // std::cout << "time: " << t << ", index: " << index << std::endl;
  const int i = index * order_;
  const double t_2 = t * t;
  const double t_3 = t_2 * t;
  const double t_4 = t_3 * t;
  const double t_5 = t_4 * t;

  PathData data;
  data.time = t;
  if (order_ == 4) {
    data.pos.x() = curve_[0](i) * t_3 + curve_[0](i + 1) * t_2 + curve_[0](i + 2) * t + curve_[0](i + 3);
    data.pos.y() = curve_[1](i) * t_3 + curve_[1](i + 1) * t_2 + curve_[1](i + 2) * t + curve_[1](i + 3);
    data.pos.z() = curve_[2](i) * t_3 + curve_[2](i + 1) * t_2 + curve_[2](i + 2) * t + curve_[2](i + 3);
    data.vel.x() = 3 * curve_[0](i) * t_2 + 2 * curve_[0](i + 1) * t + curve_[0](i + 2);
    data.vel.y() = 3 * curve_[1](i) * t_2 + 2 * curve_[1](i + 1) * t + curve_[1](i + 2);
    data.vel.z() = 3 * curve_[2](i) * t_2 + 2 * curve_[2](i + 1) * t + curve_[2](i + 2);
  } else if (order_ == 6) {
    data.pos.x() = curve_[0](i) * t_5 + curve_[0](i + 1) * t_4 + curve_[0](i + 2) * t_3 + curve_[0](i + 3) * t_2 +
                   curve_[0](i + 4) * t + curve_[0](i + 5);
    data.pos.y() = curve_[1](i) * t_5 + curve_[1](i + 1) * t_4 + curve_[1](i + 2) * t_3 + curve_[1](i + 3) * t_2 +
                   curve_[1](i + 4) * t + curve_[1](i + 5);
    data.pos.z() = curve_[2](i) * t_5 + curve_[2](i + 1) * t_4 + curve_[2](i + 2) * t_3 + curve_[2](i + 3) * t_2 +
                   curve_[2](i + 4) * t + curve_[2](i + 5);
    data.vel.x() = 5 * curve_[0](i) * t_4 + 4 * curve_[0](i + 1) * t_3 + 3 * curve_[0](i + 2) * t_2 +
                   2 * curve_[0](i + 3) * t + curve_[0](i + 4);
    data.vel.y() = 5 * curve_[1](i) * t_4 + 4 * curve_[1](i + 1) * t_3 + 3 * curve_[1](i + 2) * t_2 +
                   2 * curve_[1](i + 3) * t + curve_[1](i + 4);
    data.vel.z() = 5 * curve_[2](i) * t_4 + 4 * curve_[2](i + 1) * t_3 + 3 * curve_[2](i + 2) * t_2 +
                   2 * curve_[2](i + 3) * t + curve_[2](i + 4);
  }
  return data;
}

int PathOptimization::BinarySearch(double time) {
  int left = 0, right = len_path_ - 1;
  // 0 12 12
  // 1 11 12
  // 2 10 12
  // 3 9  12
  // 4 8  12
  // 5 7  12
  // 6 6  12
  // 7 5  12
  while (left <= right) {
    // std::cout << "left: " << left << ", right: " << right << std::endl;
    int mid = left + (right - left) / 2;
    if (time_stamp_[mid] <= time) {
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }
  // std::cout << "left: " << left << ", right: " << right << std::endl;

  return (left >= len_path_) ? len_path_ - 2 : left - 1;
}

std::vector<PathData> PathOptimization::GetReferencePath(double time, double dt, double N) {
  std::vector<PathData> path_ref;
  for (int i = 1; i <= N; ++i) {
    double t = time + i * dt;
    path_ref.emplace_back(GetOptimizedPath(t));
  }
  return path_ref;
}
