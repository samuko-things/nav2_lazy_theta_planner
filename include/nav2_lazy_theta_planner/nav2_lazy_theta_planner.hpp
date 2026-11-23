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

/**
 * @brief Represents a grid node used by the Lazy Theta* search algorithm.
 *
 * Stores the node's grid coordinates, path cost, heuristic cost,
 * queue status, and parent pointer for path reconstruction.
 */
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

/**
 * @brief Comparator for the priority queue used by the OPEN list.
 *
 * Orders nodes according to their estimated total cost:
 * f(n) = g(n) + h(n).
 *
 * Nodes with lower estimated total costs are given higher priority.
 */
struct CompareNode
{
  bool operator()(const GridNode* a, const GridNode* b) const
  {
    return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
  }
};

/**
 * @brief Represents a movement direction between neighboring grid cells.
 *
 * Stores the relative movement in the X and Y directions and
 * the corresponding movement distance.
 */
struct Dir
{ int dx; 
  int dy; 
  double dist; 
};

/**
 * @brief Stores costmap metadata required by the planner.
 *
 * Contains map resolution, inverse resolution, origin coordinates,
 * and map dimensions. These values are used for coordinate conversion
 * and grid boundary checks.
 */
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

  /**
   * @brief Configures the planner plugin.
   *
   * Initializes the planner, retrieves parameters, and establishes
   * references to the ROS 2 node, TF buffer, and costmap.
   *
   * @param parent Lifecycle node that hosts the planner.
   * @param name Name of the planner plugin.
   * @param tf TF buffer used for coordinate transformations.
   * @param costmap_ros ROS 2 costmap interface.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  
  /**
   * @brief Cleans up the planner's resources.
   */
  void cleanup() override;

  /**
   * @brief Activates the planner.
   */
  void activate() override;

  /**
   * @brief Deactivates the planner.
   */
  void deactivate() override;

  /**
   * @brief Computes a global path between the start and goal poses.
   *
   * Converts the start and goal poses into grid coordinates,
   * executes the Lazy Theta* search, reconstructs the path,
   * and applies the required path processing.
   *
   * @param start Starting pose.
   * @param goal Goal pose.
   * @param cancel_checker Callback used to cancel the planning operation.
   *
   * @return Generated path between the start and goal poses.
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  
  /**
   * @brief Executes the Lazy Theta* search algorithm.
   *
   * Performs grid-based any-angle path planning using lazy
   * line-of-sight checking and returns the goal node when
   * a valid path is found.
   *
   * @param start_node Starting grid node.
   * @param goal_node Goal grid node.
   * @param cancel_checker Callback used to cancel the search.
   * @param char_map Costmap data.
   * @param size_x Horizontal size of the costmap.
   *
   * @return Goal node if a path is found, nullptr otherwise.
   */
  GridNode* runLazyTheta(
    GridNode* start_node,
    GridNode* goal_node,
    const std::function<bool()>& cancel_checker,
    const unsigned char* char_map,
    unsigned int size_x);

  /**
   * @brief Converts a world-coordinate pose into a grid node.
   *
   * @param pose Input pose in world coordinates.
   * @return Corresponding grid node.
   */
  GridNode poseToGrid(const geometry_msgs::msg::Pose & pose) const;

  /**
   * @brief Converts a grid node into a world-coordinate pose.
   *
   * @param grid Input grid node.
   * @return Corresponding world-coordinate pose.
   */
  geometry_msgs::msg::Pose gridToPose(const GridNode & grid) const;

  /**
   * @brief Converts a grid node into a linear costmap index.
   *
   * @param grid Grid node.
   * @return Linear index of the corresponding costmap cell.
   */
  int gridToMapIndex(const GridNode & grid) const;

  /**
   * @brief Converts grid coordinates into a linear costmap index.
   *
   * @param x X-coordinate of the grid cell.
   * @param y Y-coordinate of the grid cell.
   * @return Linear index of the corresponding costmap cell.
   */
  int gridToMapIndex(const int x, const int y) const;

  /**
   * @brief Checks whether a grid node lies within the map boundaries.
   *
   * @param grid Grid node to check.
   * @return True if the node is within the map boundaries.
   */
  bool isGridOnMap(const GridNode & grid) const;

  /**
   * @brief Checks whether grid coordinates lie within the map boundaries.
   *
   * @param x X-coordinate of the grid cell.
   * @param y Y-coordinate of the grid cell.
   * @return True if the coordinates are within the map boundaries.
   */
  bool isGridOnMap(const int x, const int y) const;

  /**
   * @brief Checks whether a grid node is traversable.
   *
   * @param grid Grid node to check.
   * @param char_map Costmap data.
   * @return True if the grid node is traversable.
   */
  bool isMapCellFree(const GridNode & grid, const unsigned char* char_map) const;

  /**
   * @brief Checks whether grid coordinates are traversable.
   *
   * @param x X-coordinate of the grid cell.
   * @param y Y-coordinate of the grid cell.
   * @param char_map Costmap data.
   * @return True if the grid cell is traversable.
   */
  bool isMapCellFree(const int x, const int y, const unsigned char* char_map) const;

  /**
   * @brief Retrieves the traversal cost of a grid node.
   *
   * @param grid Grid node.
   * @param char_map Costmap data.
   * @return Traversal cost associated with the grid node.
   */
  double getGridCost(const GridNode & grid, const unsigned char* char_map) const;

  /**
   * @brief Calculates the Euclidean distance between two grid nodes.
   *
   * @param a First grid node.
   * @param b Second grid node.
   * @return Euclidean distance between the nodes.
   */
  double euclidean_distance(const GridNode & a, const GridNode & b) const;

  /**
   * @brief Checks whether a line-of-sight connection exists.
   *
   * @param current Pointer to the current grid node.
   * @param previous Pointer to the previous or ancestor grid node.
   * @param char_map Costmap data.
   * @param size_x Horizontal size of the costmap.
   * @return True if a valid line-of-sight connection exists.
   */
  bool lineOfSight(
    GridNode *current,
    GridNode *previous,
    const unsigned char* char_map,
    unsigned int size_x) const;

  /**
   * @brief Generates intermediate poses along a straight line.
   *
   * @param start Starting pose.
   * @param end Ending pose.
   * @param resolution Spacing used to generate intermediate poses.
   * @return Vector of generated poses.
   */
  std::vector<geometry_msgs::msg::PoseStamped>
  addStraightLinePoses(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    double resolution) const;

  /**
   * @brief Densifies a path by adding intermediate poses.
   *
   * @param path Input path.
   * @param goal Goal pose.
   * @return Densified path.
   */
  nav_msgs::msg::Path densifyPath(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /**
   * @brief Smooths a generated path.
   *
   * @param path Input path.
   * @param window_size Smoothing window size.
   * @return Smoothed path.
   */
  nav_msgs::msg::Path smoothPath(
    const nav_msgs::msg::Path & path,
    int window_size=5) const;

  /**
   * @brief Fills up a path with additional poses.
   *
   * @param path Input path.
   * @param goal Goal pose.
   * @return Path containing additional poses.
   */
  nav_msgs::msg::Path fillUpPath(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /**
   * @brief Retrieves a grid node from the node pool.
   *
   * Returns a node corresponding to the specified grid coordinates,
   * reusing allocated nodes where possible.
   *
   * @param x X-coordinate of the grid node.
   * @param y Y-coordinate of the grid node.
   * @return Pointer to the corresponding grid node.
   */
  GridNode* get_node_from_pool(int x, int y);

  /**
   * @brief ROS 2 costmap interface used by the planner.
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /**
   * @brief Cached costmap metadata.
   *
   * Stores map resolution, origin, and dimensions for
   * coordinate conversion and grid operations.
   */
  CostmapMeta costmap_meta_;

  /**
   * @brief Pool of grid nodes used during path search.
   *
   * Nodes are reused to reduce memory allocation overhead
   * during repeated search operations.
   */
  std::vector<GridNode> node_pool_;

  /**
   * @brief Stores visitation identifiers for grid nodes.
   *
   * Used to track node usage across search iterations.
   */
  std::vector<int> node_visited_id_;

  /**
   * @brief Identifier for the current search iteration.
   */
  int run_id_ = 0;

  /**
   * @brief OPEN list containing nodes awaiting expansion.
   *
   * Nodes are prioritized according to their estimated total cost:
   * f(n) = g(n) + h(n).
   */
  std::priority_queue<
    GridNode*,
    std::vector<GridNode*>,
    CompareNode
  > open_queue_;

  /**
   * @brief Clears the OPEN list.
   *
   * Resets the priority queue after search completion
   * or cancellation.
   */
  void clearQueue(){
    open_queue_ = std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode>();
  };

  /**
   * @brief Maximum allowable cost for line-of-sight shortcuts.
   *
   * Controls shortcut behavior through the inflation layer.
   */
  int los_shortcut_cost_limit_;

  /**
   * @brief Multiplier applied to traversal costs.
   *
   * Used to penalize traversal through higher-cost regions
   * of the costmap.
   */
  double cost_travel_multiplier_;

  /**
   * @brief Enables or disables path smoothing.
   *
   * 1 enables smoothing; 0 disables smoothing.
   */
  int smooth_path_ = 1;

  /**
   * @brief Defines the eight possible movement directions.
   *
   * Includes four cardinal directions and four diagonal directions.
   * Each direction stores its relative grid movement and distance.
   */
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

  /**
   * @brief Logger used for planner-related logging and diagnostics.
   */
  rclcpp::Logger logger_{rclcpp::get_logger("LazyThetaPlanner")};

};

}  // namespace nav2_lazy_theta_planner

#endif  // NAV2_LAZY_THETA_PLANNER_HPP_