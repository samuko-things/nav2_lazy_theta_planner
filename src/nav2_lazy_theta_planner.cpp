#include "nav2_lazy_theta_planner/nav2_lazy_theta_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>

namespace nav2_lazy_theta_planner
{

double round_to_3dp(double val) {
    return std::round(val * 1000.0) / 1000.0;
}


void LazyThetaPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node");
  }

  logger_ = node->get_logger();
  costmap_ros_ = costmap_ros;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".los_shortcut_cost_limit", rclcpp::ParameterValue(5));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_travel_multiplier", rclcpp::ParameterValue(3.0));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(1));

  // Retrieve values
  node->get_parameter(name + ".los_shortcut_cost_limit", los_shortcut_cost_limit_);
  node->get_parameter(name + ".cost_travel_multiplier", cost_travel_multiplier_);
  node->get_parameter(name + ".smooth_path", smooth_path_);

  los_shortcut_cost_limit_ = std::clamp(los_shortcut_cost_limit_, 1, 100);
  smooth_path_ = std::clamp(smooth_path_, 0, 1);

  RCLCPP_INFO_STREAM(
    logger_,
    "Configured Test Planner Plugin with: " <<
    "\n  los_shortcut_cost_limit  : " << los_shortcut_cost_limit_ <<
    "\n  cost_travel_multiplier   : " << cost_travel_multiplier_ <<
    "\n  smooth_path   : " << (smooth_path_ == 0 ? "false" : "true")
  );
}





void LazyThetaPlanner::activate()
{
  if (!costmap_ros_) {
    return;
  }

  auto costmap = costmap_ros_->getCostmap();
  costmap_meta_.update(costmap);

  unsigned int map_size = costmap_meta_.size_x * costmap_meta_.size_y;

  // Pre-allocate vector pools once on activation
  node_pool_.resize(map_size);
  node_visited_id_.resize(map_size);

  RCLCPP_INFO_STREAM(logger_, "Plugin Activated Successfully");
}




void LazyThetaPlanner::deactivate() {}




void LazyThetaPlanner::cleanup()
{
  node_pool_.clear();
  node_pool_.shrink_to_fit();

  node_visited_id_.clear();
  node_visited_id_.shrink_to_fit();

  costmap_ros_.reset();

  RCLCPP_INFO_STREAM(logger_, "Plugin Cleaned Up Successfully");
}





nav_msgs::msg::Path LazyThetaPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // auto start_time = std::chrono::steady_clock::now();

  if (!costmap_ros_) {
    return nav_msgs::msg::Path();
  }

  auto costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    RCLCPP_ERROR(logger_, "Costmap not available");
    return nav_msgs::msg::Path();
  }

  // Lock the costmap mutex to ensure thread-safe access to raw map memory during string pulling
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Atomically refresh metadata struct
  costmap_meta_.update(costmap);

  unsigned int current_map_size = costmap_meta_.size_x * costmap_meta_.size_y;

  // Fallback memory check in case the costmap dynamically resizes during runtime
  if (node_pool_.size() != current_map_size) {
    node_pool_.resize(current_map_size);
    node_visited_id_.resize(current_map_size);
  }

  // --- FAST FLAT MEMORY RESETS ---
  std::fill(node_visited_id_.begin(),node_visited_id_.end(), 0);
  const unsigned char* char_map = costmap->getCharMap();
  run_id_++;

  // Start & Goal Node Setup
  GridNode raw_start = poseToGrid(start.pose);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y);

  GridNode raw_goal = poseToGrid(goal.pose);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y);

  // Execute Search

  GridNode* best_goal;

  best_goal = runLazyTheta(
    start_node, 
    goal_node,
    cancel_checker,
    char_map, 
    costmap_meta_.size_x
  );

  // Path Reconstruction
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  if (!best_goal) {
    RCLCPP_INFO_STREAM(logger_, "No Path Generated");
    return path;
  }

  GridNode* node = best_goal;
  while (node)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = path.header.frame_id;
    ps.pose = gridToPose(*node);
    path.poses.push_back(ps);

    if (node->parent == node) {
      break;
    }
    node = node->parent;
  }

  std::reverse(path.poses.begin(), path.poses.end());

  // auto end_time = std::chrono::steady_clock::now();
  // std::chrono::duration<double> diff_sec = end_time - start_time;
  // RCLCPP_INFO_STREAM(logger_, "planning_time = " << round_to_3dp(diff_sec.count()*1000.0) << " ms");

  return fillUpPath(path, goal);

}






GridNode* LazyThetaPlanner::runLazyTheta(
  GridNode* start_node,
  GridNode* goal_node,
  const std::function<bool()>& cancel_checker,
  const unsigned char* char_map,
  unsigned int size_x)
{

  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->is_in_queue = true;
  start_node->parent = start_node;

  open_queue_.push(start_node);

  while (!open_queue_.empty())
  {
    if (cancel_checker && cancel_checker()) {
      clearQueue();
      return nullptr;
    }

    GridNode* current = open_queue_.top();
    open_queue_.pop();

    current->is_in_queue = false;

    if (current->parent && current->parent->parent && current->parent != current)
    {
      GridNode* grand_parent = current->parent->parent;
      if (lineOfSight(current, grand_parent, char_map, size_x))
      {
        double current_g_cost = grand_parent->g_cost + (euclidean_distance(*current, *grand_parent) * getGridCost(*current, char_map));
        if (current_g_cost < current->g_cost)
        {
          current->parent = grand_parent;
          current->g_cost = current_g_cost;
        }
      }
    }

    if (current->x == goal_node->x && current->y == goal_node->y) {
      clearQueue();
      return current;
    }

    for (const auto & d : dirs)
    {
      int nx = current->x + d.dx;
      int ny = current->y + d.dy;

      GridNode* nbr_node = get_node_from_pool(nx, ny);

      if (isGridOnMap(*nbr_node) && isMapCellFree(*nbr_node, char_map)) {
        double g_cost = current->g_cost + (d.dist * getGridCost(*nbr_node, char_map));
        double h_cost = euclidean_distance(*nbr_node, *goal_node);
        double f_cost = g_cost + h_cost;

        if ((nbr_node->g_cost + nbr_node->h_cost) > f_cost)
        {
          nbr_node->g_cost = g_cost;
          nbr_node->h_cost = h_cost;
          nbr_node->parent = current;

          if(!nbr_node->is_in_queue){
            nbr_node->is_in_queue = true;
            open_queue_.push(nbr_node);
          }
        }
      }
    }

  }

  return nullptr;
}





GridNode* LazyThetaPlanner::get_node_from_pool(int x, int y) {
  int index = gridToMapIndex(x, y);
  GridNode* node = &node_pool_[index];
  if (node_visited_id_[index] != run_id_) 
  {
    node->x = x;
    node->y = y;
    node->g_cost = std::numeric_limits<double>::infinity();
    node->h_cost = 0.0;
    node->parent = nullptr;
    node->is_in_queue = false;

    node_visited_id_[index] = run_id_;
  }
  return node;
};




GridNode LazyThetaPlanner::poseToGrid(const geometry_msgs::msg::Pose &pose) const
{
  int gx = static_cast<int>((pose.position.x - costmap_meta_.origin_x) * costmap_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - costmap_meta_.origin_y) * costmap_meta_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose LazyThetaPlanner::gridToPose(const GridNode &grid) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * costmap_meta_.resolution + costmap_meta_.origin_x;
  pose.position.y = grid.y * costmap_meta_.resolution + costmap_meta_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int LazyThetaPlanner::gridToMapIndex(const GridNode &grid_node) const
{
  return static_cast<int>(grid_node.y * costmap_meta_.size_x + grid_node.x);
}

int LazyThetaPlanner::gridToMapIndex(const int x, const int y) const
{
  return static_cast<int>(y * costmap_meta_.size_x + x);
}

bool LazyThetaPlanner::isGridOnMap(const GridNode &grid) const
{
  return (grid.x >= 0 && grid.x < costmap_meta_.size_x &&
          grid.y >= 0 && grid.y < costmap_meta_.size_y);
}

bool LazyThetaPlanner::isGridOnMap(const int x, const int y) const
{
  return (x >= 0 && x < costmap_meta_.size_x &&
          y >= 0 && y < costmap_meta_.size_y);
}

bool LazyThetaPlanner::isMapCellFree(const GridNode &grid, const unsigned char* char_map) const
{
  return (char_map[gridToMapIndex(grid)] < static_cast<unsigned char>(los_shortcut_cost_limit_+120));
}

bool LazyThetaPlanner::isMapCellFree(const int x, const int y, const unsigned char* char_map) const
{
  return (char_map[gridToMapIndex(x, y)] < static_cast<unsigned char>(los_shortcut_cost_limit_+120));
}

double LazyThetaPlanner::getGridCost(const GridNode &grid, const unsigned char* char_map) const
{
  return  1.0+(cost_travel_multiplier_ * std::clamp(static_cast<double>(char_map[gridToMapIndex(grid)]) / 252.0, 0.0, 1.0));
}

double LazyThetaPlanner::euclidean_distance(const GridNode &a, const GridNode &b) const{
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}


bool LazyThetaPlanner::lineOfSight(
  GridNode *current,
  GridNode *previous,
  const unsigned char* char_map,
  unsigned int size_x) const
{
  int x0 = current->x, y0 = current->y;
  int x1 = previous->x, y1 = previous->y;

  int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int cx = x0, cy = y0, e = dx - dy;

  int max_x = static_cast<int>(costmap_meta_.size_x);
  int max_y = static_cast<int>(costmap_meta_.size_y);

  const auto threshold = static_cast<unsigned char>(los_shortcut_cost_limit_);

  auto isSafe = [&](int x, int y) -> bool {
    if (x < 0 || x >= max_x || y < 0 || y >= max_y) {
      return false;
    }
    int idx = y * static_cast<int>(size_x) + x;
    return char_map[idx] <= threshold;
  };

  while (cx != x1 || cy != y1) {
    if (!isSafe(cx, cy)) {
      return false;
    }

    int e2 = 2 * e;
    if (e2 > -dy && e2 <= dx) {
      if (!isSafe(cx + sx, cy) || !isSafe(cx, cy + sy)) {
        return false;
      }
      cx += sx;
      cy += sy;
      e += dx - dy;
    } else if (e2 > -dy) {
      cx += sx;
      e -= dy;
    } else {
      cy += sy;
      e += dx;
    }
  }

  // Final check for the destination node (x1, y1)
  return isSafe(x1, y1);
}


std::vector<geometry_msgs::msg::PoseStamped>
LazyThetaPlanner::addStraightLinePoses(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & end,
  double resolution) const
{
  std::vector<geometry_msgs::msg::PoseStamped> out;

  double dx = end.pose.position.x - start.pose.position.x;
  double dy = end.pose.position.y - start.pose.position.y;
  double dist = std::hypot(dx, dy);

  if (dist == 0)
    return {};

  int steps = std::max(1, static_cast<int>(dist / resolution));

  for (int i = 1; i <= steps; i++)
  {
    geometry_msgs::msg::PoseStamped ps = start;
    ps.pose.position.x = start.pose.position.x + dx * (i / (double)steps);
    ps.pose.position.y = start.pose.position.y + dy * (i / (double)steps);
    out.push_back(ps);
  }

  return out;
}



nav_msgs::msg::Path
LazyThetaPlanner::densifyPath(
  const nav_msgs::msg::Path & path, 
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path dense_path;
  dense_path.header = path.header;

  if (path.poses.empty())
    return dense_path;

  // 1. Interpolate and densify the positions
  dense_path.poses.push_back(path.poses.front());
  for (size_t i = 1; i < path.poses.size(); i++)
  {
    auto seg = addStraightLinePoses(
      path.poses[i - 1],
      path.poses[i],
      costmap_meta_.resolution);

    dense_path.poses.insert(dense_path.poses.end(), seg.begin(), seg.end());
  }

  // 2. Calculate Yaw Orientations for intermediate waypoints
  for (size_t i = 0; i < dense_path.poses.size() - 1; i++)
  {
    double dx = dense_path.poses[i+1].pose.position.x - dense_path.poses[i].pose.position.x;
    double dy = dense_path.poses[i+1].pose.position.y - dense_path.poses[i].pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    dense_path.poses[i].pose.orientation.x = q.x();
    dense_path.poses[i].pose.orientation.y = q.y();
    dense_path.poses[i].pose.orientation.z = q.z();
    dense_path.poses[i].pose.orientation.w = q.w();
  }

  // 3. Force the absolute last waypoint to match the exact goal orientation
  dense_path.poses.back().pose.orientation = goal.pose.orientation;

  return dense_path;
}


// Simple path smoother: moving average over a sliding window in XY.
// Keeps endpoints unchanged to preserve exact start and goal.
nav_msgs::msg::Path LazyThetaPlanner::smoothPath(
  const nav_msgs::msg::Path & path,
  int window_size) const
{
  nav_msgs::msg::Path smoothed = path;
  if (smoothed.poses.size() < 3 || window_size <= 1) {
    return smoothed;
  }

  // Ensure window_size is odd and at least 3
  if (window_size < 3) {
    window_size = 3;
  }
  if (window_size % 2 == 0) {
    window_size += 1;
  }

  const int half = window_size / 2;
  const size_t n = smoothed.poses.size();
  const auto original = smoothed.poses;

  // Leave first and last pose untouched
  for (size_t i = 1; i + 1 < n; ++i) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;

    const int begin = static_cast<int>(
      i > static_cast<size_t>(half) ? i - half : 0);
    const int end = static_cast<int>(std::min<size_t>(n - 1, i + half));

    for (int j = begin; j <= end; ++j) {
      sum_x += original[j].pose.position.x;
      sum_y += original[j].pose.position.y;
      ++count;
    }

    if (count > 0) {
      smoothed.poses[i].pose.position.x = sum_x / static_cast<double>(count);
      smoothed.poses[i].pose.position.y = sum_y / static_cast<double>(count);
    }
  }

  return smoothed;
}


nav_msgs::msg::Path
LazyThetaPlanner::fillUpPath(
  const nav_msgs::msg::Path & path, 
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path dense_path = densifyPath(path, goal);

  if (smooth_path_ == 1){
    dense_path = smoothPath(dense_path);

    for (size_t i = 0; i < dense_path.poses.size() - 1; i++)
    {
      double dx = dense_path.poses[i+1].pose.position.x - dense_path.poses[i].pose.position.x;
      double dy = dense_path.poses[i+1].pose.position.y - dense_path.poses[i].pose.position.y;
      double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      dense_path.poses[i].pose.orientation = tf2::toMsg(q);
    }

    dense_path.poses.back().pose.orientation = goal.pose.orientation;
  }

  return dense_path;
}



}  // namespace theta_star_smooth_planner_plugin

PLUGINLIB_EXPORT_CLASS(
  nav2_lazy_theta_planner::LazyThetaPlanner,
  nav2_core::GlobalPlanner)