#include <iostream>

#include "control/model_predictive_control.h"
#include "planning/path_optimization.h"
#include "planning/rapidly_exploring_random_tree_star.h"

void TestMPC();
void TestRRT2D();
void TestRRT3D();
void TestRRT2DAndOpt();
void TestRRT2DAndMPC();

int main(int argc, char* argv[]) {
  int test_no = atoi(argv[1]);
  switch (test_no) {
    case 0:
      TestMPC();
      break;
    case 1:
      TestRRT2D();
      break;
    case 2:
      TestRRT3D();
      break;
    case 3:
      TestRRT2DAndMPC();
      break;
    case 4:
      TestRRT2DAndOpt();
      break;
    default:
      TestRRT2D();
      break;
  }
  return 0;
}

void TestMPC() {
  // 定义小车模型和MPC参数
  CarModel car;
  car.sp_dt = 0.1;
  car.ct_dt = 0.1;
  car.width = 0.3973;
  car.radius = 0.084;
  car.v_max = 0.2;
  MPCParams params;
  params.horizon = 50;
  params.Q.setZero();
  params.Q(0, 0) = 10.0;
  params.Q(1, 1) = 10.0;
  params.Q(2, 2) = 0.01;
  params.R.setZero();
  params.R(0, 0) = 0.1;
  params.R(1, 1) = 0.1;
  params.P.setZero();
  params.P(0, 0) = 1;
  params.P(1, 1) = 1;
  params.P(2, 2) = 0.01;

  // 初始化MPC控制器
  MPCController controller(car, params);

  // 定义初始状态和目标状态
  CarStates initState;
  initState << 0.0, 0.0, 0.0, 0.0, 0.0;
  CarStates targetState;
  targetState << 1.0, 4.0, 1.0, 0.0, 0.0;

  CarStates curr_state = initState;
  // 运行MPC控制器
  for (int i = 0; i < 100000; ++i) {
    // 更新状态
    controller.UpdateState(curr_state, targetState);

    // 计算控制输入
    CarControl control = controller.GetNextControl();
    // fprintf(stdout, "Control: %2.4f %2.4f.\n", control(0), control(1));

    // 输出控制输入和下一个状态
    curr_state = controller.NextState(curr_state, control);
    fprintf(stdout, "Time %2.4f, pose %2.4f, %2.4f, %2.4f.\n", i * car.ct_dt, curr_state(0), curr_state(1),
            curr_state(2));

    if (controller.IsTerminate()) {
      fprintf(stdout, "Termination condition reached.\n");
      break;
    }
  }
}

void TestRRT2D() {
  // 定义RRT*算法参数
  double step_size = 0.5;
  double goal_threshold = 0.5;
  double x_min = 0.0;
  double x_max = 20.0;
  double y_min = 0.0;
  double y_max = 20.0;
  double z_min = 0.0;
  double z_max = 0.0;
  double near_radius = 2.0;
  int max_iterations = 100000;

  // 定义起点和终点
  Node start{0.0, 0.0, 0.0, 0, -1, 0.0};
  Node goal{10.0, 10.0, 0.0, max_iterations, -1, 0.0};

  // 定义障碍物
  std::vector<Obstacle> obstacles{{5.0, 5.0, 0.0, 2.0}, {7.0, 7.0, 0.0, 1.0}};

  // 定义RRT*算法实例
  RRTStar rrt(start, goal, obstacles, step_size, goal_threshold, near_radius, x_min, x_max, y_min, y_max, z_min, z_max);

  // 调用RRT*算法函数
  rrt.Run(max_iterations);

  // 获取路径结果
  std::vector<Node> path = rrt.GetPath();

  // 输出路径
  std::cout << "RRT Path:(" << path.size() << ") ";
  for (int i = 0; i < path.size(); i++) {
    rrt.logs_ << path[i].x << ", " << path[i].y << ", " << path[i].z << std::endl;
    std::cout << "(" << path[i].self_id << ", " << path[i].x << ", " << path[i].y << ", " << path[i].z << ")";
    if (i < path.size() - 1) {
      std::cout << " -> ";
    }
  }
  std::cout << std::endl;
}

void TestRRT3D() {
  // 定义RRT*算法参数
  double step_size = 0.5;
  double goal_threshold = 0.5;
  double x_min = 0.0;
  double x_max = 20.0;
  double y_min = 0.0;
  double y_max = 20.0;
  double z_min = 0.0;
  double z_max = 20.0;
  double near_radius = 2.0;
  int max_iterations = 100000;

  // 定义起点和终点
  Node start{0.0, 0.0, 0.0, 0, -1, 0.0};
  Node goal{10.0, 10.0, 10.0, max_iterations, -1, 0.0};

  // 定义障碍物
  std::vector<Obstacle> obstacles{{5.0, 5.0, 5.0, 2.0}, {7.0, 7.0, 7.0, 1.0}};

  // 定义RRT*算法实例
  RRTStar rrt(start, goal, obstacles, step_size, goal_threshold, near_radius, x_min, x_max, y_min, y_max, z_min, z_max);

  // 调用RRT*算法函数
  rrt.Run(max_iterations);

  // 获取路径结果
  std::vector<Node> path = rrt.GetPath();

  // 输出路径
  std::cout << "RRT Path:(" << path.size() << ") ";
  for (int i = 0; i < path.size(); i++) {
    rrt.logs_ << path[i].x << ", " << path[i].y << ", " << path[i].z << std::endl;
    std::cout << "(" << path[i].self_id << ", " << path[i].x << ", " << path[i].y << ", " << path[i].z << ")";
    if (i < path.size() - 1) {
      std::cout << " -> ";
    }
  }
  std::cout << std::endl;
}

void TestRRT2DAndMPC() {
  // 定义RRT*算法参数
  double step_size = 0.5;
  double goal_threshold = 0.5;
  double x_min = 0.0;
  double x_max = 20.0;
  double y_min = 0.0;
  double y_max = 20.0;
  double z_min = 0.0;
  double z_max = 0.0;
  double near_radius = 2.0;
  int max_iterations = 100000;

  // 定义起点和终点
  Node start{0.0, 0.0, 0.0, 0, -1, 0.0};
  Node goal{10.0, 10.0, 0.0, max_iterations, -1, 0.0};

  // 定义障碍物
  std::vector<Obstacle> obstacles{{5.0, 5.0, 0.0, 2.0}, {7.0, 7.0, 0.0, 1.0}};

  // 定义RRT*算法实例
  RRTStar rrt(start, goal, obstacles, step_size, goal_threshold, near_radius, x_min, x_max, y_min, y_max, z_min, z_max);

  // 调用RRT*算法函数
  rrt.Run(max_iterations);

  // 获取路径结果
  std::vector<Node> path = rrt.GetPath();

  // 输出路径
  std::cout << "RRT Path:(" << path.size() << ") ";
  for (int i = 0; i < path.size(); i++) {
    rrt.logs_ << path[i].x << ", " << path[i].y << ", " << path[i].z << std::endl;
    std::cout << "(" << path[i].self_id << ", " << path[i].x << ", " << path[i].y << ", " << path[i].z << ")";
    if (i < path.size() - 1) {
      std::cout << " -> ";
    }
  }
  std::cout << std::endl;

  // 定义小车模型和MPC参数
  CarModel car;
  car.sp_dt = 0.1;
  car.ct_dt = 0.1;
  car.width = 0.3973;
  car.radius = 0.084;
  car.v_max = 0.2;
  MPCParams params;
  params.horizon = 50;
  params.Q.setZero();
  params.Q(0, 0) = 10.0;
  params.Q(1, 1) = 10.0;
  params.Q(2, 2) = 0.01;
  params.R.setZero();
  params.R(0, 0) = 0.1;
  params.R(1, 1) = 0.1;
  params.P.setZero();
  params.P(0, 0) = 1;
  params.P(1, 1) = 1;
  params.P(2, 2) = 0.01;

  // 初始化MPC控制器
  MPCController controller(car, params);

  for (int i = 0; i < path.size(); i++) {
    path[i].z = std::atan2(path[i].y, path[i].x);
  }

  // 定义初始状态和目标状态
  CarStates initState;
  initState << 0.0, 0.0, 0.0, 0.0, 0.0;
  CarStates targetState;
  targetState << path[0].x, path[0].y, path[0].z, 0.0, 0.0;

  CarStates curr_state = initState;
  int path_id = 0;
  // 运行MPC控制器
  for (int i = 0; i < 10000; ++i) {
    // 更新状态
    controller.UpdateState(curr_state, targetState);

    // 计算控制输入
    CarControl control = controller.GetNextControl();
    // fprintf(stdout, "Control: %2.4f %2.4f.\n", control(0), control(1));

    // 输出控制输入和下一个状态
    curr_state = controller.NextState(curr_state, control);
    fprintf(stdout, "Time %2.4f, pose %2.4f, %2.4f, %2.4f.\n", i * car.ct_dt, curr_state(0), curr_state(1),
            curr_state(2));

    if (controller.IsTerminate() || (i % 100 == 0)) {
      ++path_id;
      if (path_id >= path.size()) {
        fprintf(stdout, "Termination condition reached.\n");
        break;
      }
      targetState(0) = path[path_id].x;
      targetState(1) = path[path_id].y;
      targetState(2) = std::atan2(path[path_id].y - curr_state(1), path[path_id].x - curr_state(0));
      // fprintf(stdout, "New target: %2.4f %2.4f %2.4f.\n", targetState(0), targetState(1), targetState(2));
    }
  }
}

void TestRRT2DAndOpt() {
  // 定义RRT*算法参数
  double step_size = 0.5;
  double goal_threshold = 0.5;
  double x_min = 0.0;
  double x_max = 20.0;
  double y_min = 0.0;
  double y_max = 20.0;
  double z_min = 0.0;
  double z_max = 0.0;
  double near_radius = 2.0;
  int max_iterations = 100000;

  // 定义起点和终点
  Node start{0.0, 0.0, 0.0, 0, -1, 0.0};
  Node goal{10.0, 10.0, 0.0, max_iterations, -1, 0.0};

  // 定义障碍物
  std::vector<Obstacle> obstacles{{5.0, 5.0, 0.0, 2.0}, {7.0, 7.0, 0.0, 1.0}};

  // 定义RRT*算法实例
  RRTStar rrt(start, goal, obstacles, step_size, goal_threshold, near_radius, x_min, x_max, y_min, y_max, z_min, z_max);

  // 调用RRT*算法函数
  rrt.Run(max_iterations);

  // 获取路径结果
  std::vector<Node> path = rrt.GetPath();

  // 输出路径
  std::cout << "RRT Path:(" << path.size() << ") ";
  for (int i = 0; i < path.size(); i++) {
    rrt.logs_ << path[i].x << ", " << path[i].y << ", " << path[i].z << std::endl;
    std::cout << "(" << path[i].self_id << ", " << path[i].x << ", " << path[i].y << ", " << path[i].z << ")";
    if (i < path.size() - 1) {
      std::cout << " -> ";
    }
  }
  std::cout << std::endl;

  // 输出带Yaw角的路径
  for (int i = 0; i < path.size(); i++) {
    path[i].z = std::atan2(path[i].y, path[i].x);
  }
  std::cout << "OPT Path:(" << path.size() << ") ";
  for (int i = 0; i < path.size(); i++) {
    std::cout << "(" << path[i].x << ", " << path[i].y << ", " << path[i].z << ")";
    if (i < path.size() - 1) {
      std::cout << " -> ";
    }
  }
  std::cout << std::endl;

  // 轨迹优化
  std::vector<PathData> trajectory;
  std::vector<PathData> opt_trajectory;
  PathData data;
  for (int i = 0; i < path.size(); ++i) {
    data.time = 0.0;
    data.pos << path[i].x, path[i].y, path[i].z;
    trajectory.emplace_back(data);
    // fprintf(stdout, "raw: %2.4f %2.4f %2.4f %2.4f.\n", data.time, data.pos(0), data.pos(1), data.pos(2));
  }

  const double vel_set = 1.0;
  for (int i = 1; i < trajectory.size(); ++i) {
    double distance = (trajectory[i].pos - trajectory[i - 1].pos).norm();
    trajectory[i].time = distance / vel_set + trajectory[i - 1].time;
    fprintf(stdout, "raw: %2.4f %2.4f %2.4f %2.4f.\n", trajectory[i].time, trajectory[i].pos(0), trajectory[i].pos(1),
            trajectory[i].pos(2));
  }

  PathOptimization opt(5);
  opt.OptimizePath(trajectory);
  opt.SaveOptimizedPath(trajectory);
  for (double i = 0; i < trajectory.size() - 1; i += 0.1) {
    data = opt.GetOptimizedPath(i);
    opt_trajectory.emplace_back(data);
    fprintf(stdout, "opt: %2.4f %2.4f %2.4f %2.4f.\n", data.time, data.pos(0), data.pos(1), data.pos(2));
  }

  // 轨迹跟踪
  // 定义小车模型和MPC参数
  CarModel car;
  car.sp_dt = 0.1;
  car.ct_dt = 0.1;
  car.width = 0.3973;
  car.radius = 0.084;
  car.v_max = 0.2;
  MPCParams params;
  params.horizon = 10;
  params.Q.setZero();
  params.Q(0, 0) = 10.0;
  params.Q(1, 1) = 10.0;
  params.Q(2, 2) = 0.01;
  params.R.setZero();
  params.R(0, 0) = 0.1;
  params.R(1, 1) = 0.1;
  params.P.setZero();
  params.P(0, 0) = 10;
  params.P(1, 1) = 10;
  params.P(2, 2) = 0.01;

  // 初始化MPC控制器
  MPCController controller(car, params);

  // 定义初始状态和目标状态
  CarStates initState;
  initState << 0.0, 0.0, 0.0, 0.0, 0.0;
  CarStates targetState;
  targetState << opt_trajectory[1].pos, 0.0, 0.0;

  CarStates curr_state = initState;
  std::vector<PathData> path_ref;
  int path_id = 0;
  // 运行MPC控制器
  double cur_time = 0.0;
  double ref_time = 0.0;
  for (int i = 0; i < 10000000; ++i) {
    // 获取参考
    path_ref = opt.GetReferencePath(cur_time, car.sp_dt, params.horizon);
    ref_time = cur_time + car.sp_dt * params.horizon;
    targetState << path_ref.back().pos, 0.0, 0.0;

    // 更新参考
    controller.SetRefPath(path_ref);

    // 更新状态
    controller.UpdateState(curr_state, targetState);

    // 计算控制输入
    CarControl control = controller.GetNextControl();
    // fprintf(stdout, "Control: %2.4f %2.4f.\n", control(0), control(1));

    // 输出控制输入和下一个状态
    curr_state = controller.NextState(curr_state, control);
    fprintf(stdout, "Time %2.4f(%2.4f), pose %2.4f(%2.4f), %2.4f(%2.4f), %2.4f(%2.4f).\n", cur_time, ref_time,
            curr_state(0), targetState(0), curr_state(1), targetState(1), curr_state(2), targetState(2));

    // 更新下一步跟踪的点
    if (controller.IsTerminate() || (i % 100 == 0)) {
      cur_time += car.sp_dt;
      // fprintf(stdout, "New target: %2.4f %2.4f %2.4f %2.4f.\n", time, targetState(0), targetState(1),
      // targetState(2));
    }

    // 是否退出
    if (ref_time > opt_trajectory.back().time + car.sp_dt) {
      break;
    }
  }
}
