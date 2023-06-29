#include "RRTStar.h"

RRTStar::RRTStar(const Node& start, const Node& goal, const std::vector<Obstacle>& obstacles, double step_size,
                 double goal_threshold, double near_radius, double x_min, double x_max, double y_min, double y_max,
                 double z_min, double z_max) {
  start_ = start;
  goal_ = goal;
  obstacles_ = obstacles;
  step_size_ = step_size;
  goal_threshold_ = goal_threshold;
  near_radius_ = near_radius;
  x_min_ = x_min;
  x_max_ = x_max;
  y_min_ = y_min;
  y_max_ = y_max;
  z_min_ = z_min;
  z_max_ = z_max;
  nodes_.emplace_back(start_);

  logs_.open("../data/data.txt");
}

void RRTStar::Run(int max_iterations) {
  for (int i = 1; i < max_iterations; i++) {
    // 生成一个随机节点
    Node random_node = GetRandomNode(i);
    // 找到距离这个随机节点最近的一个已知节点
    Node nearest_node = GetNearestNode(random_node);

    // 将最近的已知节点作为随机节点的父节点.
    // 扩展树的时候，允许的最大步长是step_size，如果超过步长，那就直接按比例缩放到步长内
    Node new_node = Extend(nearest_node, random_node);
    if (!IsCollisionFree(nearest_node, new_node)) {
      // std::cout << i << " is collision and it will be remove: " << new_node.x << ", " << new_node.y << ", "
      //           << new_node.z << std::endl;
      --i;
      continue;
    }
    if (new_node.x == nearest_node.x && new_node.y == nearest_node.y && new_node.z == nearest_node.z) {
      continue;
    }
    // 新节点cost = 父节点（最近节点）的cost + 新节点到父节点的cost（距离)
    double new_cost = nearest_node.cost + GetDistance(new_node, nearest_node);

    // 获取以新节点为圆心，规定半径内的近邻节点
    std::vector<Node*> near_nodes = GetNearNodes(new_node);
    int min_cost_parent_id = nearest_node.self_id;
    double min_cost = new_cost;

    if (min_cost_parent_id == -1) {
      new_node.cost = min_cost;
      new_node.parent_id = min_cost_parent_id;
    } else {
      for (auto near_node : near_nodes) {
        double cost = near_node->cost + GetDistance(new_node, *near_node);
        // cost小于之前新节点的cost，并且无碰撞，那就把近邻节点作为新节点的父节点
        if (IsCollisionFree(*near_node, new_node) && cost < min_cost) {
          min_cost_parent_id = near_node->self_id;
          min_cost = cost;
          // fprintf(stdout, "Change parent id for node %d.\n", i);
        }
      }
      // 为新节点指定cost最小的父节点
      new_node.cost = min_cost;
      new_node.parent_id = min_cost_parent_id;
    }
    nodes_.emplace_back(new_node);

    // RRT树重新布线：判断是否可以将新节点作为近邻节点的父节点
    for (auto near_node : near_nodes) {
      double cost = new_cost + GetDistance(*near_node, new_node);
      if (IsCollisionFree(new_node, *near_node) && cost < near_node->cost) {
        near_node->cost = cost;
        near_node->parent_id = new_node.self_id;
      }
    }

    // 判断是否终止
    if (IsTerminated(new_node)) {
      goal_.self_id = i + 1;
      goal_.parent_id = new_node.self_id;
      nodes_.emplace_back(goal_);
      fprintf(stdout, "Terminate reached with %ld steps.\n", nodes_.size());
      return;
    }
  }
}

// 获取路径
std::vector<Node> RRTStar::GetPath() {
  // ViewAllNodes();
  double min_distance_to_goal = 1e5;
  int path_id = 0;
  for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it) {
    if (GetDistance(*it, goal_) < min_distance_to_goal) {
      min_distance_to_goal = GetDistance(*it, goal_);
      path_id = it->self_id;
    }
  }

  std::vector<Node> path;
  while (path_id != -1) {
    path.emplace_back(nodes_[path_id]);
    // std::cout << path_id << ": " << nodes_[path_id].x << ", " << nodes_[path_id].y << ", " << nodes_[path_id].z
    //           << std::endl;
    path_id = nodes_[path_id].parent_id;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

Node RRTStar::GetRandomNode(int self_id) const {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(x_min_, x_max_);
  std::uniform_real_distribution<> dis_y(y_min_, y_max_);
  std::uniform_real_distribution<> dis_z(z_min_, z_max_);
  return {dis_x(gen), dis_y(gen), dis_z(gen), self_id, -1, 0.0};
}

Node RRTStar::GetNearestNode(const Node& query_node) {
  Node nearest_node = nodes_[0];
  double min_distance = GetDistance(nearest_node, query_node);
  for (int i = 1; i < nodes_.size(); i++) {
    Node node = nodes_[i];
    double distance = GetDistance(node, query_node);
    if (distance < min_distance) {
      nearest_node = node;
      min_distance = distance;
    }
  }
  return nearest_node;
}

Node RRTStar::Extend(const Node& from_node, const Node& to_node) {
  double distance = GetDistance(from_node, to_node);
  if (distance <= step_size_) {
    return to_node;

  } else {
    double ratio = step_size_ / distance;
    double x = from_node.x + ratio * (to_node.x - from_node.x);
    double y = from_node.y + ratio * (to_node.y - from_node.y);
    double z = from_node.z + ratio * (to_node.z - from_node.z);
    return {x, y, z, to_node.self_id, -1, 0.0};
  }
}

// 获取附近的节点
std::vector<Node*> RRTStar::GetNearNodes(const Node& node) {
  std::vector<Node*> near_nodes;
  for (int i = 0; i < nodes_.size(); i++) {
    if (GetDistance(node, nodes_[i]) <= near_radius_ && IsCollisionFree(node, nodes_[i])) {
      near_nodes.emplace_back(&nodes_[i]);
    }
  }
  return near_nodes;
}

// 判断是否碰撞
bool RRTStar::IsCollisionFree(const Node& from_node, const Node& to_node) const {
  for (const auto obstacle : obstacles_) {
    if (GetDistance({obstacle.x, obstacle.y, obstacle.z}, from_node) <= obstacle.radius ||
        GetDistance({obstacle.x, obstacle.y, obstacle.z}, to_node) <= obstacle.radius) {
      return false;
    }
  }
  return true;
}

// 获取两个节点之间的距离
double RRTStar::GetDistance(const Node& node1, const Node& node2) const {
  return std::sqrt(std::pow(node1.x - node2.x, 2.0) + std::pow(node1.y - node2.y, 2.0) +
                   std::pow(node1.z - node2.z, 2.0));
}

bool RRTStar::IsTerminated(const Node& node) const { return GetDistance(node, goal_) < goal_threshold_; }

void RRTStar::ViewAllNodes() const {
  for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
    fprintf(stdout, "Node %d: %d, %2.4f, %2.4f, %2.4f.\n", it->self_id, it->parent_id, it->x, it->y, it->z);
  }
}

int main() {
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
  std::cout << "Path: ";
  for (int i = 0; i < path.size(); i++) {
    rrt.logs_ << path[i].x << ", " << path[i].y << ", " << path[i].z << std::endl;
    std::cout << "(" << path[i].self_id << ", " << path[i].x << ", " << path[i].y << ", " << path[i].z << ")";
    if (i < path.size() - 1) {
      std::cout << " -> ";
    }
  }
  std::cout << std::endl;

  return 0;
}