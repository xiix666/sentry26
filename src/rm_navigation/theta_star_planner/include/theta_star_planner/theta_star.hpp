#ifndef THETA_STAR_PLANNER__THETA_STAR_HPP_
#define THETA_STAR_PLANNER__THETA_STAR_HPP_

#include <cmath>
#include <chrono>
#include <vector>
#include <queue>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <limits>
#include <cmath>
const double INF_COST = DBL_MAX;
const int UNKNOWN_COST = 255;
const int OCCUPIED_COST = 254;
const int MAX_NON_OBSTACLE_COST = 252;

struct coordsM
{
  int x, y;
};

struct coordsW
{
  double x, y;
};

struct tree_node
{
  int x, y;
  double g = INF_COST;
  double h = INF_COST;
  const tree_node * parent_id = nullptr;
  bool is_in_queue = false;
  double f = INF_COST;
};

struct comp
{
  bool operator()(const tree_node * p1, const tree_node * p2)
  {
    return (p1->f) > (p2->f);
  }
};

namespace theta_star
{
class ThetaStar
{
public:
  coordsM src_{}, dst_{};
  nav2_costmap_2d::Costmap2D * costmap_{};

  double w_traversal_cost_;
  double w_euc_cost_;
  double w_heuristic_cost_;
  int how_many_corners_;
  bool allow_unknown_;
  int size_x_, size_y_;

  ThetaStar();

  ~ThetaStar() = default;

  bool generatePath(std::vector<coordsW> & raw_path);

  inline bool isSafe(const int & cx, const int & cy) const
  {
    if (hasObstacleAround(cx, cy)) {
      return false;
    }
    if (isConditionalAreaBlockedCell(cx, cy)) {
      return false;
    }
    return (costmap_->getCost(cx, cy) == UNKNOWN_COST && allow_unknown_) ||
           costmap_->getCost(cx, cy) <= MAX_NON_OBSTACLE_COST;
  }
  inline bool hasObstacleAround(const int & cx, const int & cy) const
  {
    constexpr int obstacle_check_radius = 1;
  
    for (int dx = -obstacle_check_radius; dx <= obstacle_check_radius; ++dx) {
      for (int dy = -obstacle_check_radius; dy <= obstacle_check_radius; ++dy) {
        const int nx = cx + dx;
        const int ny = cy + dy;

        if (
          nx < 0 || ny < 0 ||
          nx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
          ny >= static_cast<int>(costmap_->getSizeInCellsY()))
        {
          return true;
        }
  
        const unsigned char raw_cost = costmap_->getCost(nx, ny);
  
        if (raw_cost == UNKNOWN_COST) {
          if (!allow_unknown_) {
            return true;
          }
          continue;
        }
  
        if (raw_cost > MAX_NON_OBSTACLE_COST) {
          return true;
        }
      }
    }
  
    return false;
  }

  void setStartAndGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  bool isUnsafeToPlan() const
  {
    return !(isSafe(src_.x, src_.y)) || !(isSafe(dst_.x, dst_.y));
  }
  bool findNearestFreeCell(
  const coordsM & src,
  coordsM & result,
  const int max_radius) const;
  int nodes_opened = 0;
  bool moveStartAndGoalToNearestFreeCell(const int max_radius);

  void setConditionalForbiddenArea(
    bool enabled,
    double x1,
    double y1,
    double x2,
    double y2);

  bool isConditionalAreaAllowedForCurrentPlan() const
  {
    return conditional_area_allowed_for_plan_;
  }
protected:
  std::vector<tree_node *> node_position_;

  std::vector<tree_node> nodes_data_;

  std::priority_queue<tree_node *, std::vector<tree_node *>, comp> queue_;

  int index_generated_;

  const coordsM moves[8] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};

  tree_node * exp_node;

  void resetParent(tree_node * curr_data);

  void setNeighbors(const tree_node * curr_data);

  bool losCheck(
    const int & x0, const int & y0, const int & x1, const int & y1,
    double & sl_cost) const;

  void backtrace(std::vector<coordsW> & raw_points, const tree_node * curr_n) const;

  bool isSafe(const int & cx, const int & cy, double & cost) const
  {
    if (hasObstacleAround(cx, cy)) {
      return false;
    }
    if (isConditionalAreaBlockedCell(cx, cy)) {
      return false;
    }
    double curr_cost = getCost(cx, cy);
    if ((costmap_->getCost(cx, cy) == UNKNOWN_COST && allow_unknown_) ||
      curr_cost <= MAX_NON_OBSTACLE_COST)
    {
      if (costmap_->getCost(cx, cy) == UNKNOWN_COST) {
        curr_cost = OCCUPIED_COST - 1;
      }
      cost += w_traversal_cost_ * curr_cost * curr_cost / MAX_NON_OBSTACLE_COST /
        MAX_NON_OBSTACLE_COST;
      return true;
    } else {
      return false;
    }
  }

  inline double getCost(const int & cx, const int & cy) const
  {
    return 26 + 0.9 * costmap_->getCost(cx, cy);
  }

  inline double getTraversalCost(const int & cx, const int & cy)
  {
    double curr_cost = getCost(cx, cy);
    return w_traversal_cost_ * curr_cost * curr_cost / MAX_NON_OBSTACLE_COST /
           MAX_NON_OBSTACLE_COST;
  }

  inline double getEuclideanCost(const int & ax, const int & ay, const int & bx, const int & by)
  {
    return w_euc_cost_ * std::hypot(ax - bx, ay - by);
  }

  inline double getHCost(const int & cx, const int & cy)
  {
    return w_heuristic_cost_ * std::hypot(cx - dst_.x, cy - dst_.y);
  }

  inline bool withinLimits(const int & cx, const int & cy) const
  {
    return cx >= 0 && cx < size_x_ && cy >= 0 && cy < size_y_;
  }

  inline bool isGoal(const tree_node & this_node) const
  {
    return this_node.x == dst_.x && this_node.y == dst_.y;
  }

  void initializePosn(int size_inc = 0);

  inline void addIndex(const int & cx, const int & cy, tree_node * node_this)
  {
    node_position_[size_x_ * cy + cx] = node_this;
  }

  inline tree_node * getIndex(const int & cx, const int & cy)
  {
    return node_position_[size_x_ * cy + cx];
  }

  void addToNodesData(const int & id_this)
  {
    if (static_cast<int>(nodes_data_.size()) <= id_this) {
      nodes_data_.push_back({});
    } else {
      nodes_data_[id_this] = {};
    }
  }

  void resetContainers();

  void clearQueue()
  {
    queue_ = std::priority_queue<tree_node *, std::vector<tree_node *>, comp>();
  }

  bool isInsideConditionalAreaWorld(
    double wx,
    double wy,
    double min_x,
    double max_x,
    double min_y,
    double max_y) const;

  bool isConditionalAreaBlockedCell(
    int mx,
    int my) const;

  bool conditional_area_enabled_{true};

  double conditional_area_min_x_{12.5};
  double conditional_area_max_x_{13.5};
  double conditional_area_min_y_{-2.6};
  double conditional_area_max_y_{1.0};

  double conditional_area2_min_x_{19.0};
  double conditional_area2_max_x_{20.7};
  double conditional_area2_min_y_{-1.2};
  double conditional_area2_max_y_{0.8};

  bool conditional_area_allowed_for_plan_{false};

  bool conditional_area2_allowed_for_plan_{false};
};
}

#endif
