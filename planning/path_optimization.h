#ifndef PLANNING_PATH_OPTIMIZATION_H_
#define PLANNING_PATH_OPTIMIZATION_H_

#include <Eigen/Dense>

struct PathData {
  double time;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
};

class PathOptimization {
 public:
  PathOptimization(int order);
  ~PathOptimization();

  void OptimizePath(const std::vector<PathData>& path);

  void SaveOptimizedPath(const std::vector<PathData>& path);

  PathData GetOptimizedPath(double time);

  void ResetOrder(int order) { order_ = order + 1; }

 private:
  int order_;
  int len_path_;
  Eigen::VectorXd time_stamp_;
  Eigen::VectorXd curve_[3];

  int BinarySearch(double time);
};

#endif  // PLANNING_PATH_OPTIMIZATION_H_
