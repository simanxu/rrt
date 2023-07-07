#ifndef CONTROL_MODEL_PREDICTIVE_CONTROL_H_
#define CONTROL_MODEL_PREDICTIVE_CONTROL_H_
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <qpOASES.hpp>

#include "planning/path_optimization.h"

// 定义小车模型
struct CarModel {
  double sp_dt;   // 采样时间
  double ct_dt;   // 控制时间
  double width;   // 两轮间距
  double radius;  // 轮子半径
  double v_max;   // 轮子转速上限
};

// 定义MPC参数
struct MPCParams {
  int horizon;        // 控制时域长度
  Eigen::Matrix3d Q;  // 状态代价矩阵
  Eigen::Matrix2d R;  // 控制代价矩阵
  Eigen::Matrix3d P;  // 终端代价矩阵
};

// 定义问题维度和时间步长，终止条件
const int n = 3;  // 状态变量数量
const int m = 2;  // 控制变量数量
const double states_threshold = 0.002;
const double terninate_threshold = 0.05;
using CarStates = Eigen::Matrix<double, 5, 1>;
using CarControl = Eigen::Matrix<double, 2, 1>;

// 定义MPC控制器
class MPCController {
 public:
  MPCController(const CarModel& car, const MPCParams& params);
  ~MPCController();

  void UpdateState(const CarStates& state, const CarStates& target);

  CarControl GetNextControl();

  CarStates NextState(const CarStates& state, const CarControl& control);

  bool IsTerminate();

  void SetRefPath(const std::vector<PathData>& path);

 private:
  int states_terminate_count_;
  std::ofstream logs_;
  CarModel car_;
  MPCParams params_;
  CarStates states_;
  CarStates states_prev_;
  CarStates target_;
  CarControl control_;
  std::vector<PathData> path_ref_;
};

#endif  // CONTROL_MODEL_PREDICTIVE_CONTROL_H_
