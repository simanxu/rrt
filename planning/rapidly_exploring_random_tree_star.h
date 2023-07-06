#ifndef PLANNING_RAPIDLY_EXPLORING_RANDOM_TREE_STAR_H_
#define PLANNING_RAPIDLY_EXPLORING_RANDOM_TREE_STAR_H_

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

// Define Node struct
struct Node {
  double x;
  double y;
  double z;

  int self_id;
  int parent_id;

  double cost;
};

// Define Obstacle struct
struct Obstacle {
  double x;
  double y;
  double z;
  double radius;
};

// Define RRTStar class
class RRTStar {
 public:
  RRTStar(const Node& start, const Node& goal, const std::vector<Obstacle>& obstacles, double step_size,
          double goal_threshold, double near_radius, double x_min, double x_max, double y_min, double y_max,
          double z_min, double z_max);

  void Run(int max_iterations);

  std::vector<Node> GetPath();

  std::ofstream logs_;

 private:
  Node GetRandomNode(int self_id) const;

  Node GetNearestNode(const Node& query_node);

  Node Extend(const Node& from_node, const Node& to_node);

  std::vector<int> GetNearNodes(const Node& node);

  bool IsCollisionFree(const Node& from_node, const Node& to_node) const;

  double GetDistance(const Node& node1, const Node& node2) const;

  bool IsTerminated(const Node& node) const;

  void ViewAllNodes() const;

  Node start_;
  Node goal_;
  std::vector<Obstacle> obstacles_;
  std::vector<Node> nodes_;
  double step_size_;
  double goal_threshold_;
  double near_radius_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
};

#endif  // PLANNING_RAPIDLY_EXPLORING_RANDOM_TREE_STAR_H_
