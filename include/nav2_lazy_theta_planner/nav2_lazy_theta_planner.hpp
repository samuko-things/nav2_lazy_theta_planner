#ifndef NAV2_LAZY_THETA_PLANNER_HPP_
#define NAV2_LAZY_THETA_PLANNER_HPP_

#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

namespace nav2_lazy_theta_planner
{

struct GridNode
{
  int x{0};
  int y{0};

  double g_cost{0.0};
  double h_cost{0.0};
  bool is_in_queue{false};
  GridNode *parent{nullptr};

  GridNode(int _x = 0, int _y = 0) : x(_x), y(_y) {}
};

struct CompareNode
{
  bool operator()(const GridNode* a, const GridNode* b) const
  {
    return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
  }
};

struct Dir
{ int dx; 
  int dy; 
  double dist; 
};


struct CostmapMeta {
  double resolution{0.0};
  double inv_resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int size_x{0};
  int size_y{0};

  // Helper to update all fields atomically from a costmap pointer
  void update(const nav2_costmap_2d::Costmap2D* costmap) {
    if (!costmap) return;
    resolution = costmap->getResolution();
    inv_resolution = (resolution > 0.0) ? (1.0 / resolution) : 0.0;
    origin_x = costmap->getOriginX();
    origin_y = costmap->getOriginY();
    size_x = static_cast<int>(costmap->getSizeInCellsX());
    size_y = static_cast<int>(costmap->getSizeInCellsY());
  }
};


class LazyThetaPlanner : public nav2_core::GlobalPlanner
{
public:
  LazyThetaPlanner() = default;
  ~LazyThetaPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:

  GridNode* runLazyTheta(
    GridNode* start_node,
    GridNode* goal_node,
    const std::function<bool()>& cancel_checker,
    const unsigned char* char_map,
    unsigned int size_x);

  GridNode poseToGrid(const geometry_msgs::msg::Pose &pose) const;
  geometry_msgs::msg::Pose gridToPose(const GridNode &grid) const;
  int gridToMapIndex(const GridNode &grid) const;
  int gridToMapIndex(const int x, const int y) const;
  bool isGridOnMap(const GridNode &grid) const;
  bool isGridOnMap(const int x, const int y) const;
  bool isMapCellFree(const GridNode &grid, const unsigned char* char_map) const;
  bool isMapCellFree(const int x, const int y, const unsigned char* char_map) const;
  double getGridCost(const GridNode &grid, const unsigned char* char_map) const;
  double euclidean_distance(const GridNode &a, const GridNode &b) const;

  bool lineOfSight(
    GridNode *current,
    GridNode *previous,
    const unsigned char* char_map,
    unsigned int size_x) const;

  std::vector<geometry_msgs::msg::PoseStamped>
  addStraightLinePoses(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    double resolution) const;

  nav_msgs::msg::Path densifyPath(
    const nav_msgs::msg::Path & path, 
    const geometry_msgs::msg::PoseStamped & goal) const;

  nav_msgs::msg::Path smoothPath(
    const nav_msgs::msg::Path & path,
    int window_size=5) const;

  nav_msgs::msg::Path fillUpPath(
    const nav_msgs::msg::Path & path, 
    const geometry_msgs::msg::PoseStamped & goal) const;

  GridNode* get_node_from_pool(int x, int y);

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  CostmapMeta costmap_meta_;
  
  std::vector<GridNode> node_pool_; 
  std::vector<int> node_visited_id_;
  int run_id_ = 0;

  std::priority_queue<
    GridNode*,
    std::vector<GridNode*>,
    CompareNode
  > open_queue_;

  void clearQueue(){
    open_queue_ = std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode>();
  };

  int los_shortcut_cost_limit_;
  double cost_travel_multiplier_;
  int smooth_path_ = 1;

  Dir dirs[8] = {
    {-1,  0, 1.0},
    { 1,  0, 1.0},
    { 0, -1, 1.0},
    { 0,  1, 1.0},
    {-1, -1, 1.4142},
    {-1,  1, 1.4142},
    { 1, -1, 1.4142},
    { 1,  1, 1.4142}
  };

  rclcpp::Logger logger_{rclcpp::get_logger("LazyThetaPlanner")};

};

}  // namespace nav2_lazy_theta_planner

#endif  // NAV2_LAZY_THETA_PLANNER_HPP_