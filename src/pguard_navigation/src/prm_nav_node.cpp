/
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/PathGeometric.h>

#include <vector>
#include <set>
#include <cmath>
#include <mutex>
#include <memory>
#include <limits>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <future>
#include <optional>
#include <unordered_map>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace pearlguard_navigation
{

// ----------------------------------------------------------------------------
// Constantes de configuration partagees
// ----------------------------------------------------------------------------
namespace cost
{
constexpr double kFreeSpace          = 5.0;
constexpr double kTraversable        = 8.0;
constexpr double kGoalAcceptable     = 15.0;
constexpr double kStartAcceptable    = 20.0;
constexpr double kReplanTrigger      = 10.0;
constexpr double kLethal             = 100.0;
}  // namespace cost

// ----------------------------------------------------------------------------
// Fonctions utilitaires de grille
// ----------------------------------------------------------------------------
inline int coordsToIndex(
  double x, double y, int width, double resolution, double offset_x, double offset_y)
{
  const int grid_x = static_cast<int>(std::round((x - offset_x) / resolution));
  const int grid_y = static_cast<int>(std::round((y - offset_y) / resolution));
  return grid_y * width + grid_x;
}

inline void indexToCoords(
  int index, int width, double resolution, double offset_x, double offset_y,
  double & x, double & y)
{
  x = (index % width) * resolution + offset_x;
  y = (index / width) * resolution + offset_y;
}

inline bool isCollisionFree(int index, const std::vector<double> & costmap, int width, int height)
{
  if (index < 0 || index >= static_cast<int>(costmap.size())) {
    return false;
  }
  const int x = index % width;
  const int y = index / width;
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return false;
  }
  return costmap[index] < cost::kTraversable;
}

inline double computePathLength(
  const std::vector<int> & path, int width, double resolution, double offset_x, double offset_y)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    double x1, y1, x2, y2;
    indexToCoords(path[i - 1], width, resolution, offset_x, offset_y, x1, y1);
    indexToCoords(path[i], width, resolution, offset_x, offset_y, x2, y2);
    length += std::hypot(x2 - x1, y2 - y1);
  }
  return length;
}

inline double computeSafeCostThreshold(double safety_buffer, double inflation_radius)
{
  if (safety_buffer >= inflation_radius) {
    RCLCPP_ERROR(
      rclcpp::get_logger("prm_nav"),
      "safety_buffer_ (%.2f) >= inflation_radius_ (%.2f): degagement non garanti, "
      "augmentez inflation_radius_ dans les parametres.",
      safety_buffer, inflation_radius);
    return 1.01;
  }
  const double threshold = 15.0 * (1.0 - safety_buffer / inflation_radius);
  return std::max(1.01, threshold);
}

inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// ----------------------------------------------------------------------------
// Verification de securite ponctuelle (coordonnees continues)
// ----------------------------------------------------------------------------
inline bool isXYSafe(
  double x, double y, int width, int height, double resolution,
  double offset_x, double offset_y, const std::vector<double> & costmap,
  double safety_margin_cost)
{
  const int idx = coordsToIndex(x, y, width, resolution, offset_x, offset_y);
  if (idx < 0 || idx >= static_cast<int>(costmap.size())) {
    return false;
  }
  const int cx = idx % width;
  const int cy = idx / width;
  if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
    return false;
  }
  return costmap[idx] <= safety_margin_cost;
}


inline bool findNearestSafeCell(
  double x, double y, int width, int height, double resolution,
  double offset_x, double offset_y, const std::vector<double> & costmap,
  double safety_margin_cost, double max_search_radius, double & out_x, double & out_y)
{
  if (isXYSafe(x, y, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost)) {
    out_x = x;
    out_y = y;
    return true;
  }
  const int max_cells = std::max(1, static_cast<int>(std::ceil(max_search_radius / resolution)));
  for (int r = 1; r <= max_cells; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;  // uniquement le contour de l'anneau courant (evite de retester le centre)
        }
        const double cx = x + dx * resolution;
        const double cy = y + dy * resolution;
        if (isXYSafe(cx, cy, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost)) {
          out_x = cx;
          out_y = cy;
          return true;
        }
      }
    }
  }
  return false;
}

inline std::vector<std::pair<double, double>> chaikinSmoothSafe(
  const std::vector<std::pair<double, double>> & pts, int iterations, double ratio,
  int width, int height, double resolution, double offset_x, double offset_y,
  const std::vector<double> & costmap, double safety_margin_cost)
{
  auto current = pts;
  for (int it = 0; it < iterations; ++it) {
    if (current.size() < 3) {
      break;
    }
    std::vector<std::pair<double, double>> next;
    next.reserve(current.size() * 2);
    next.push_back(current.front());

    for (size_t i = 0; i + 1 < current.size(); ++i) {
      const auto & p0 = current[i];
      const auto & p1 = current[i + 1];
      double qx = p0.first + ratio * (p1.first - p0.first);
      double qy = p0.second + ratio * (p1.second - p0.second);
      double rx = p0.first + (1.0 - ratio) * (p1.first - p0.first);
      double ry = p0.second + (1.0 - ratio) * (p1.second - p0.second);

      double safe_qx = qx, safe_qy = qy;
      if (!isXYSafe(qx, qy, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost)) {
        if (!findNearestSafeCell(
            qx, qy, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost,
            resolution * 3.0, safe_qx, safe_qy))
        {
          safe_qx = p0.first;
          safe_qy = p0.second;
        }
      }
      double safe_rx = rx, safe_ry = ry;
      if (!isXYSafe(rx, ry, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost)) {
        if (!findNearestSafeCell(
            rx, ry, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost,
            resolution * 3.0, safe_rx, safe_ry))
        {
          safe_rx = p1.first;
          safe_ry = p1.second;
        }
      }
      next.emplace_back(safe_qx, safe_qy);
      next.emplace_back(safe_rx, safe_ry);
    }
    next.push_back(current.back());
    current = std::move(next);
  }
  return current;
}

class CubicSpline
{
public:
  CubicSpline(const std::vector<double> & x, const std::vector<double> & y)
  {
    const size_t n = x.size();
    if (n < 2 || x.size() != y.size()) {
      throw std::invalid_argument("CubicSpline: tailles x/y invalides");
    }

    std::vector<double> h(n - 1), alpha(n - 1), l(n), mu(n), z(n);
    a_ = y;
    b_.resize(n - 1);
    c_.resize(n);
    d_.resize(n - 1);

    for (size_t i = 0; i < n - 1; ++i) {
      h[i] = x[i + 1] - x[i];
      if (h[i] <= 0.0) {
        throw std::invalid_argument("CubicSpline: valeurs x non strictement croissantes");
      }
    }

    for (size_t i = 1; i < n - 1; ++i) {
      alpha[i] = 3.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }

    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (size_t i = 1; i < n - 1; ++i) {
      l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c_[n - 1] = 0.0;

    for (int j = static_cast<int>(n) - 2; j >= 0; --j) {
      c_[j] = z[j] - mu[j] * c_[j + 1];
      b_[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
      d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
    }
  }

  double interpolate(double t, const std::vector<double> & x_orig) const
  {
    if (x_orig.empty()) {
      return 0.0;
    }
    const size_t n = x_orig.size();
    size_t i = 0;
    for (; i < n - 1; ++i) {
      if (t <= x_orig[i + 1]) {
        break;
      }
    }
    if (i >= n - 1) {
      i = n - 2;
    }
    const double dt = t - x_orig[i];
    return a_[i] + b_[i] * dt + c_[i] * dt * dt + d_[i] * dt * dt * dt;
  }

private:
  std::vector<double> a_, b_, c_, d_;
};

// ----------------------------------------------------------------------------
// Verificateur de validite d'etat OMPL
// ----------------------------------------------------------------------------
class CostmapValidityChecker : public ob::StateValidityChecker
{
public:
  CostmapValidityChecker(
    const ob::SpaceInformationPtr & si, const std::vector<double> & costmap,
    int width, int height, double resolution, double offset_x, double offset_y,
    double safe_cost_threshold)
  : ob::StateValidityChecker(si),
    costmap_(costmap),
    width_(width),
    height_(height),
    resolution_(resolution),
    offset_x_(offset_x),
    offset_y_(offset_y),
    safe_cost_threshold_(safe_cost_threshold)
  {}

  bool isValid(const ob::State * state) const override
  {
    const auto * rvs = state->as<ob::RealVectorStateSpace::StateType>();
    if (!rvs) {
      return false;
    }
    const double x = rvs->values[0];
    const double y = rvs->values[1];
    const int index = coordsToIndex(x, y, width_, resolution_, offset_x_, offset_y_);
    if (index < 0 || index >= static_cast<int>(costmap_.size())) {
      return false;
    }
    return costmap_[index] < safe_cost_threshold_;
  }

private:
  const std::vector<double> & costmap_;
  int width_, height_;
  double resolution_, offset_x_, offset_y_;
  double safe_cost_threshold_;
};

// ----------------------------------------------------------------------------
// Objectif d'optimisation : penalise la proximite aux obstacles (clearance)
// ----------------------------------------------------------------------------
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
  ClearanceObjective(
    const ob::SpaceInformationPtr & si, const std::vector<double> & costmap,
    int width, double resolution, double offset_x, double offset_y)
  : ob::StateCostIntegralObjective(si, true),
    costmap_(costmap),
    width_(width),
    resolution_(resolution),
    offset_x_(offset_x),
    offset_y_(offset_y)
  {}

  ob::Cost stateCost(const ob::State * s) const override
  {
    const auto * rvs = s->as<ob::RealVectorStateSpace::StateType>();
    const int idx = coordsToIndex(rvs->values[0], rvs->values[1], width_, resolution_, offset_x_, offset_y_);
    const double c = (idx >= 0 && idx < static_cast<int>(costmap_.size())) ? costmap_[idx] : cost::kLethal;
    return ob::Cost(c);
  }

private:
  const std::vector<double> & costmap_;
  int width_;
  double resolution_, offset_x_, offset_y_;
};

// ----------------------------------------------------------------------------
// Planification globale par PRM (OMPL)
// ----------------------------------------------------------------------------
std::vector<int> planPRM(
  int start_index, int goal_index, int width, int height,
  const std::vector<double> & costmap, double resolution,
  double offset_x, double offset_y, double solve_time,
  double safe_cost_threshold, double clearance_weight = 0.6)
{
  std::vector<int> path;

  if (costmap.empty() || start_index < 0 || goal_index < 0 ||
    start_index >= static_cast<int>(costmap.size()) ||
    goal_index >= static_cast<int>(costmap.size()))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("prm_nav"),
      "Entree invalide: start_index=%d, goal_index=%d, costmap_size=%zu",
      start_index, goal_index, costmap.size());
    return path;
  }

  double start_x, start_y, goal_x, goal_y;
  indexToCoords(start_index, width, resolution, offset_x, offset_y, start_x, start_y);
  indexToCoords(goal_index, width, resolution, offset_x, offset_y, goal_x, goal_y);

  auto space = std::make_shared<ob::RealVectorStateSpace>(2);
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, offset_x);
  bounds.setHigh(0, offset_x + width * resolution);
  bounds.setLow(1, offset_y);
  bounds.setHigh(1, offset_y + height * resolution);
  space->setBounds(bounds);

  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker(
    std::make_shared<CostmapValidityChecker>(
      si, costmap, width, height, resolution, offset_x, offset_y, safe_cost_threshold));

  const double max_extent = space->getMaximumExtent();
  const double desired_step_m = resolution / 2.0;
  const double validity_fraction =
    (max_extent > 1e-9) ? (desired_step_m / max_extent) : 0.01;
  si->setStateValidityCheckingResolution(validity_fraction);

  auto pdef = std::make_shared<ob::ProblemDefinition>(si);

  ob::ScopedState<> start(space);
  start[0] = start_x;
  start[1] = start_y;
  ob::ScopedState<> goal(space);
  goal[0] = goal_x;
  goal[1] = goal_y;
  pdef->setStartAndGoalStates(start, goal, 0.2);

  auto len_obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
  auto clr_obj = std::make_shared<ClearanceObjective>(si, costmap, width, resolution, offset_x, offset_y);
  auto combined_obj = std::make_shared<ob::MultiOptimizationObjective>(si);
  combined_obj->addObjective(len_obj, 1.0);
  combined_obj->addObjective(clr_obj, clearance_weight);
  pdef->setOptimizationObjective(combined_obj);

  auto planner = std::make_shared<og::PRM>(si);
  planner->setProblemDefinition(pdef);
  planner->setMaxNearestNeighbors(8);

  const ob::PlannerStatus solved = planner->ob::Planner::solve(solve_time);

  if (!solved) {
    RCLCPP_WARN(rclcpp::get_logger("prm_nav"), "Le PRM n'a pas trouve de chemin");
    return path;
  }

  auto * path_geom = pdef->getSolutionPath()->as<og::PathGeometric>();
  if (!path_geom) {
    RCLCPP_WARN(rclcpp::get_logger("prm_nav"), "Chemin solution invalide");
    return path;
  }

  const double path_length = path_geom->length();
  const int n_interp = std::max(
    100, static_cast<int>(std::ceil(path_length / (resolution / 2.0))));
  path_geom->interpolate(n_interp);
  path.reserve(path_geom->getStateCount());
  for (size_t i = 0; i < path_geom->getStateCount(); ++i) {
    const auto * state = path_geom->getState(i)->as<ob::RealVectorStateSpace::StateType>();
    const int index = coordsToIndex(state->values[0], state->values[1], width, resolution, offset_x, offset_y);
    if (isCollisionFree(index, costmap, width, height)) {
      path.push_back(index);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("prm_nav"), "PRM: chemin trouve avec %zu noeuds", path.size());
  return path;
}

// ----------------------------------------------------------------------------
// Simplification du chemin (line-of-sight shortcutting)
// ----------------------------------------------------------------------------
std::vector<int> simplifyPath(
  const std::vector<int> & path, int width, int height, double resolution,
  double offset_x, double offset_y, const std::vector<double> & costmap,
  double safety_margin_cost = cost::kFreeSpace)
{
  if (path.size() < 3) {
    return path;
  }

  std::vector<int> simplified;
  simplified.push_back(path.front());

  constexpr double kSampleStepFraction = 0.2;

  auto isSegmentCollisionFree = [&](int idx1, int idx2) {
      double x1, y1, x2, y2;
      indexToCoords(idx1, width, resolution, offset_x, offset_y, x1, y1);
      indexToCoords(idx2, width, resolution, offset_x, offset_y, x2, y2);

      const double dist = std::hypot(x2 - x1, y2 - y1);
      const int steps = static_cast<int>(dist / (resolution * kSampleStepFraction)) + 1;
      for (int s = 0; s <= steps; ++s) {
        const double t = static_cast<double>(s) / steps;
        const double x = x1 + t * (x2 - x1);
        const double y = y1 + t * (y2 - y1);
        const int idx = coordsToIndex(x, y, width, resolution, offset_x, offset_y);
        if (idx < 0 || idx >= static_cast<int>(costmap.size())) {
          return false;
        }
        if (!isCollisionFree(idx, costmap, width, height) || costmap[idx] > safety_margin_cost) {
          return false;
        }
      }
      return true;
    };

  size_t i = 0;
  while (i < path.size() - 1) {
    size_t j = path.size() - 1;
    while (j > i + 1 && !isSegmentCollisionFree(path[i], path[j])) {
      --j;
    }
    if (j == i + 1 && !isSegmentCollisionFree(path[i], path[j])) {
      RCLCPP_ERROR(
        rclcpp::get_logger("prm_nav"),
        "Segment %zu->%zu non franchissable meme en fallback.", i, j);
    }
    simplified.push_back(path[j]);
    i = j;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("prm_nav"), "Simplification: %zu -> %zu points",
    path.size(), simplified.size());
  return simplified;
}

// ----------------------------------------------------------------------------
// Lissage par spline cubique puis re-simplification
// ----------------------------------------------------------------------------
std::vector<int> smoothPath(
  const std::vector<int> & raw_path, int width, int height, double resolution,
  double offset_x, double offset_y, const std::vector<double> & costmap,
  double safety_margin_cost = cost::kFreeSpace)
{
  if (raw_path.size() < 2) {
    return raw_path;
  }

  // Etape 1: shortcutting -- elimine le bruit dense du PRM et reduit aux points cles.
  // C'est important car la spline precedente, appliquee directement sur le chemin brut,
  const auto shortcut = simplifyPath(
    raw_path, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost);
  if (shortcut.size() < 3) {
    return shortcut;
  }

  std::vector<std::pair<double, double>> key_coords;
  key_coords.reserve(shortcut.size());
  for (const int idx : shortcut) {
    double x, y;
    indexToCoords(idx, width, resolution, offset_x, offset_y, x, y);
    key_coords.emplace_back(x, y);
  }

  // Etape 2: arrondi des angles vifs (Chaikin), avec repli securise cellule par cellule.
  // C'est ce qui traite directement le probleme des "angles aigus" y compris pres des

  const auto rounded_coords = chaikinSmoothSafe(
    key_coords, /*iterations=*/2, /*ratio=*/0.2,
    width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost);

  // Etape 3: interpolation spline cubique pour obtenir une courbe C1 continue.
  // En cas de point invalide, on cherche d'abord une cellule sure proche (au lieu de

  std::vector<double> x_coords, y_coords, t_coords;
  x_coords.reserve(rounded_coords.size());
  y_coords.reserve(rounded_coords.size());
  t_coords.reserve(rounded_coords.size());
  for (size_t i = 0; i < rounded_coords.size(); ++i) {
    x_coords.push_back(rounded_coords[i].first);
    y_coords.push_back(rounded_coords[i].second);
    t_coords.push_back(static_cast<double>(i));
  }

  CubicSpline spline_x(t_coords, x_coords);
  CubicSpline spline_y(t_coords, y_coords);

  const size_t n_points = std::max<size_t>(rounded_coords.size() * 2, 10);
  const double t_max = static_cast<double>(rounded_coords.size() - 1);
  const double dt = t_max / static_cast<double>(n_points - 1);

  std::vector<std::pair<double, double>> smoothed_coords;
  smoothed_coords.reserve(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    const double t = i * dt;
    double x = spline_x.interpolate(t, t_coords);
    double y = spline_y.interpolate(t, t_coords);

    if (!isXYSafe(x, y, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost)) {
      double safe_x, safe_y;
      if (findNearestSafeCell(
          x, y, width, height, resolution, offset_x, offset_y, costmap, safety_margin_cost,
          resolution * 4.0, safe_x, safe_y))
      {
        x = safe_x;
        y = safe_y;
      } else {
        size_t nearest_idx = static_cast<size_t>(std::round(t));
        nearest_idx = std::min(nearest_idx, rounded_coords.size() - 1);
        x = rounded_coords[nearest_idx].first;
        y = rounded_coords[nearest_idx].second;
      }
    }
    smoothed_coords.emplace_back(x, y);
  }

  // Etape 4: conversion en indices + dedoublonnage. Volontairement PAS de re-simplifyPath

  std::vector<int> smoothed_path;
  smoothed_path.reserve(smoothed_coords.size());
  int last_idx = -1;
  for (const auto & [x, y] : smoothed_coords) {
    const int idx = coordsToIndex(x, y, width, resolution, offset_x, offset_y);
    if (idx != last_idx && isCollisionFree(idx, costmap, width, height)) {
      smoothed_path.push_back(idx);
      last_idx = idx;
    }
  }

  if (smoothed_path.empty()) {
    return shortcut;
  }
  return smoothed_path;
}

// ----------------------------------------------------------------------------
// Grille spatiale d'obstacles
// ----------------------------------------------------------------------------
class ObstacleGrid
{
public:
  explicit ObstacleGrid(double cell_size)
  : cell_size_(std::max(cell_size, 0.05))
  {}

  void build(const std::vector<std::pair<double, double>> & points)
  {
    cells_.clear();
    for (const auto & p : points) {
      cells_[cellKey(p.first, p.second)].push_back(p);
    }
  }

  std::vector<std::pair<double, double>> query(double x, double y, double radius) const
  {
    std::vector<std::pair<double, double>> result;
    const int64_t cx = static_cast<int64_t>(std::floor(x / cell_size_));
    const int64_t cy = static_cast<int64_t>(std::floor(y / cell_size_));
    const int reach = static_cast<int>(std::ceil(radius / cell_size_)) + 1;
    for (int dx = -reach; dx <= reach; ++dx) {
      for (int dy = -reach; dy <= reach; ++dy) {
        const auto it = cells_.find(key(cx + dx, cy + dy));
        if (it == cells_.end()) {
          continue;
        }
        result.insert(result.end(), it->second.begin(), it->second.end());
      }
    }
    return result;
  }

  double distanceToNearest(double x, double y, double max_radius) const
  {
    double best = std::numeric_limits<double>::max();
    for (const auto & [ox, oy] : query(x, y, max_radius)) {
      best = std::min(best, std::hypot(ox - x, oy - y));
    }
    return best;
  }

private:
  int64_t cellKey(double x, double y) const
  {
    return key(
      static_cast<int64_t>(std::floor(x / cell_size_)),
      static_cast<int64_t>(std::floor(y / cell_size_)));
  }

  static int64_t key(int64_t cx, int64_t cy)
  {
    return (cx << 32) ^ (cy & 0xffffffffLL);
  }

  double cell_size_;
  std::unordered_map<int64_t, std::vector<std::pair<double, double>>> cells_;
};

// ----------------------------------------------------------------------------
// Planificateur local (Dynamic Window Approach)
// ----------------------------------------------------------------------------
class DwaLocalPlanner
{
public:
  struct Trajectory
  {
    std::vector<std::pair<double, double>> points;
    std::vector<double> yaws;
    double cost = std::numeric_limits<double>::max();
    double linear_vel = 0.0;
    double angular_vel = 0.0;
    bool valid = false;
  };

  DwaLocalPlanner(
    double max_linear_vel, double max_angular_vel, double resolution,
    double dt, int num_steps, double safety_buffer, double v_step, double w_step,
    double heading_weight, double goal_weight, double velocity_weight,
    double obstacle_soft_weight, double path_weight, double path_corridor_slack,
    double max_linear_accel, double max_angular_accel,
    double footprint_front, double footprint_back, double footprint_half_width,
    double corridor_relax_enter_dist = 0.6, double corridor_relax_exit_dist = 1.2,
    double path_deviation_hard_cap = 1.5, double footprint_safety_margin = 0.12,
    double costmap_safety_threshold = cost::kReplanTrigger)
  : max_linear_vel_(max_linear_vel),
    max_angular_vel_(max_angular_vel),
    resolution_(resolution),
    dt_(dt),
    safety_buffer_(safety_buffer),
    num_steps_(std::max(1, num_steps)),
    v_step_(std::max(v_step, 1e-3)),
    w_step_(std::max(w_step, 1e-3)),
    heading_weight_(heading_weight),
    goal_weight_(goal_weight),
    velocity_weight_(velocity_weight),
    obstacle_soft_weight_(obstacle_soft_weight),
    path_weight_(path_weight),
    path_corridor_slack_(std::max(0.0, path_corridor_slack)),
    max_linear_accel_(max_linear_accel),
    max_angular_accel_(max_angular_accel),
    footprint_front_(footprint_front),
    footprint_back_(footprint_back),
    footprint_half_width_(footprint_half_width),
    corridor_relax_enter_dist_(std::max(1e-3, corridor_relax_enter_dist)),
    corridor_relax_exit_dist_(std::max(corridor_relax_enter_dist_ + 1e-3, corridor_relax_exit_dist)),
    path_deviation_hard_cap_(std::max(path_corridor_slack_ + 1e-3, path_deviation_hard_cap)),
    footprint_safety_margin_(std::max(0.0, footprint_safety_margin)),
    costmap_safety_threshold_(costmap_safety_threshold)
  {}

  Trajectory computeBestTrajectory(
    double current_x, double current_y, double current_yaw,
    double target_x, double target_y,
    const std::vector<double> & costmap, int width, int height,
    double offset_x, double offset_y,
    const ObstacleGrid & obstacle_grid,
    double current_v, double current_w, double control_dt,
    double heading_weight_scale = 1.0,
    double goal_weight_scale = 1.0,
    const std::vector<std::pair<double, double>> & path_ref_points = {})
  {
    Trajectory best_traj;
    best_traj.cost = std::numeric_limits<double>::max();
    best_traj.valid = false;

    const double initial_goal_dist = std::hypot(target_x - current_x, target_y - current_y);

    const double v_floor = -max_linear_vel_ * 0.1;
    const double v_ceil = max_linear_vel_;
    const double w_floor = -max_angular_vel_;
    const double w_ceil = max_angular_vel_;

    const double v_min = std::clamp(current_v - max_linear_accel_ * control_dt, v_floor, v_ceil);
    const double v_max = std::clamp(current_v + max_linear_accel_ * control_dt, v_floor, v_ceil);
    const double w_min = std::clamp(current_w - max_angular_accel_ * control_dt, w_floor, w_ceil);
    const double w_max = std::clamp(current_w + max_angular_accel_ * control_dt, w_floor, w_ceil);

    const double query_radius =
      std::max(footprint_front_, std::max(footprint_back_, footprint_half_width_)) +
      footprint_safety_margin_ + 0.1;

    double corridor_clearance = std::numeric_limits<double>::max();
    if (!path_ref_points.empty()) {
      const size_t sample_stride = std::max<size_t>(1, path_ref_points.size() / 15);
      for (size_t i = 0; i < path_ref_points.size(); i += sample_stride) {
        const auto & [px, py] = path_ref_points[i];
        corridor_clearance = std::min(
          corridor_clearance, obstacle_grid.distanceToNearest(px, py, safety_buffer_ * 3.0));
      }
    } else {
      corridor_clearance = obstacle_grid.distanceToNearest(current_x, current_y, safety_buffer_ * 3.0);
    }

 
    if (!avoiding_obstacle_ && corridor_clearance < corridor_relax_enter_dist_) {
      avoiding_obstacle_ = true;
    } else if (avoiding_obstacle_ && corridor_clearance > corridor_relax_exit_dist_) {
      avoiding_obstacle_ = false;
    }

    double effective_path_weight = path_weight_;
    if (avoiding_obstacle_) {
      const double t = std::clamp(corridor_clearance / corridor_relax_exit_dist_, 0.0, 1.0);
      effective_path_weight = path_weight_ * (0.25 + 0.75 * t);
    }

    const double allowed_deviation = avoiding_obstacle_
      ? path_deviation_hard_cap_
      : std::max(path_corridor_slack_ * 1.5, 0.3);

    for (double v = v_min; v <= v_max; v += v_step_) {
      for (double w = w_min; w <= w_max; w += w_step_) {
        Trajectory traj = simulateTrajectory(current_x, current_y, current_yaw, v, w);

        if (!isTrajectorySafe(traj, costmap, width, height, offset_x, offset_y, obstacle_grid, query_radius)) {
          continue;
        }

        if (!path_ref_points.empty() &&
          computeMaxPathDeviation(traj, path_ref_points) > allowed_deviation)
        {
          continue;  // s'ecarte trop du chemin global sans justification d'obstacle reel
        }

        const auto & last = traj.points.back();
        const double final_yaw = current_yaw + w * num_steps_ * dt_;
        const double desired_yaw = std::atan2(target_y - current_y, target_x - current_x);
        const double heading_error = std::abs(normalizeAngle(desired_yaw - final_yaw));
        const double heading_cost = heading_error / M_PI;

        const double remaining_dist = std::hypot(last.first - target_x, last.second - target_y);
        const double goal_cost = (initial_goal_dist > 1e-6)
          ? std::min(1.0, remaining_dist / initial_goal_dist)
          : 0.0;

        const double obstacle_soft_cost = computeSoftObstacleCost(
          traj, costmap, width, offset_x, offset_y, obstacle_grid);

        const double path_dev_cost = computePathDeviationCost(traj, path_ref_points);

        const double scaled_heading_cost = heading_weight_scale * heading_cost;
        const double v_norm = std::clamp(v / max_linear_vel_, 0.0, 1.0);
        const double velocity_reward = v_norm * std::max(0.0, 1.0 - scaled_heading_cost);

        const double total_cost =
          heading_weight_ * scaled_heading_cost +
          goal_weight_ * goal_weight_scale * goal_cost +
          obstacle_soft_weight_ * obstacle_soft_cost +
          effective_path_weight * path_dev_cost -
          velocity_weight_ * velocity_reward;

        if (total_cost < best_traj.cost) {
          best_traj = traj;
          best_traj.cost = total_cost;
          best_traj.linear_vel = v;
          best_traj.angular_vel = w;
          best_traj.valid = true;
        }
      }
    }
    return best_traj;
  }

private:
  Trajectory simulateTrajectory(double x, double y, double yaw, double v, double w) const
  {
    Trajectory traj;
    traj.points.reserve(num_steps_ + 1);
    traj.yaws.reserve(num_steps_ + 1);
    traj.points.emplace_back(x, y);
    traj.yaws.push_back(yaw);

    for (int i = 0; i < num_steps_; ++i) {
      x += v * std::cos(yaw) * dt_;
      y += v * std::sin(yaw) * dt_;
      yaw += w * dt_;
      yaw = normalizeAngle(yaw);
      traj.points.emplace_back(x, y);
      traj.yaws.push_back(yaw);
    }
    return traj;
  }

  bool footprintHit(double rx, double ry, double ryaw, double ox, double oy) const
  {
    const double dx = ox - rx;
    const double dy = oy - ry;
    const double cos_y = std::cos(-ryaw);
    const double sin_y = std::sin(-ryaw);
    const double lx = dx * cos_y - dy * sin_y;
    const double ly = dx * sin_y + dy * cos_y;

    return (
      lx > -(footprint_back_ + footprint_safety_margin_) &&
      lx < (footprint_front_ + footprint_safety_margin_) &&
      std::abs(ly) < (footprint_half_width_ + footprint_safety_margin_));
  }

  bool isTrajectorySafe(
    const Trajectory & traj,
    const std::vector<double> & costmap, int width, int height,
    double offset_x, double offset_y,
    const ObstacleGrid & obstacle_grid, double query_radius) const
  {
    (void)height;
    for (size_t i = 0; i < traj.points.size(); ++i) {
      const auto & [x, y] = traj.points[i];
      const int idx = coordsToIndex(x, y, width, resolution_, offset_x, offset_y);
      if (idx < 0 || idx >= static_cast<int>(costmap.size())) {
        return false;
      }
      if (costmap[idx] >= costmap_safety_threshold_) {
        return false;
      }
      const double yaw = traj.yaws[i];
      for (const auto & [ox, oy] : obstacle_grid.query(x, y, query_radius)) {
        if (footprintHit(x, y, yaw, ox, oy)) {
          return false;
        }
      }
    }
    return true;
  }

  double computeSoftObstacleCost(
    const Trajectory & traj,
    const std::vector<double> & costmap, int width,
    double offset_x, double offset_y,
    const ObstacleGrid & obstacle_grid) const
  {
    double cost_sum = 0.0;
    const double search_radius = safety_buffer_ * 1.5;
    for (const auto & [x, y] : traj.points) {
      const int idx = coordsToIndex(x, y, width, resolution_, offset_x, offset_y);
      if (idx >= 0 && idx < static_cast<int>(costmap.size()) && costmap[idx] >= cost::kFreeSpace) {
        cost_sum += (costmap[idx] - cost::kFreeSpace) / (cost::kReplanTrigger - cost::kFreeSpace);
      }
      const double min_obstacle_dist = obstacle_grid.distanceToNearest(x, y, search_radius);
      if (min_obstacle_dist < search_radius) {
        const double margin = search_radius - min_obstacle_dist;
        cost_sum += margin / (safety_buffer_ * 0.5);
      }
    }
    return cost_sum / static_cast<double>(traj.points.size());
  }

  double computePathDeviationCost(
    const Trajectory & traj,
    const std::vector<std::pair<double, double>> & path_ref_points) const
  {
    if (path_ref_points.empty()) {
      return 0.0;
    }
    double cost_sum = 0.0;
    for (const auto & [x, y] : traj.points) {
      double min_dist = std::numeric_limits<double>::max();
      for (const auto & [px, py] : path_ref_points) {
        min_dist = std::min(min_dist, std::hypot(px - x, py - y));
      }
      if (min_dist > path_corridor_slack_) {

        const double excess = min_dist - path_corridor_slack_;
        cost_sum += excess * excess;
      }
    }
    return cost_sum / static_cast<double>(traj.points.size());
  }


  double computeMaxPathDeviation(
    const Trajectory & traj,
    const std::vector<std::pair<double, double>> & path_ref_points) const
  {
    double max_dist = 0.0;
    for (const auto & [x, y] : traj.points) {
      double min_dist = std::numeric_limits<double>::max();
      for (const auto & [px, py] : path_ref_points) {
        min_dist = std::min(min_dist, std::hypot(px - x, py - y));
      }
      max_dist = std::max(max_dist, min_dist);
    }
    return max_dist;
  }

  double max_linear_vel_, max_angular_vel_;
  double resolution_, dt_, safety_buffer_;
  int num_steps_;
  double v_step_, w_step_;
  double heading_weight_, goal_weight_, velocity_weight_, obstacle_soft_weight_;
  double path_weight_, path_corridor_slack_;
  double max_linear_accel_, max_angular_accel_;
  double footprint_front_, footprint_back_, footprint_half_width_;
  double corridor_relax_enter_dist_ = 0.6, corridor_relax_exit_dist_ = 1.2;
  double path_deviation_hard_cap_ = 1.5;
  double footprint_safety_margin_ = 0.12;
  double costmap_safety_threshold_ = cost::kReplanTrigger;
  bool avoiding_obstacle_ = false;
};

// ----------------------------------------------------------------------------
// Noeud principal
// ----------------------------------------------------------------------------
class PRMNode : public rclcpp::Node
{
public:
  PRMNode()
  : Node("prm_nav_node")
  {
    declareParameters();
    readParameters();
    setupPublishers();
    setupSubscribers();

    local_planner_ = std::make_unique<DwaLocalPlanner>(
      max_linear_vel_, max_angular_vel_, resolution_,
      local_plan_dt_, local_plan_steps_, safety_buffer_, dwa_v_step_, dwa_w_step_,
      dwa_heading_weight_, dwa_goal_weight_, dwa_velocity_weight_, dwa_obstacle_soft_weight_,
      dwa_path_weight_, dwa_path_corridor_slack_,
      max_linear_accel_, max_angular_accel_,
      footprint_front_, footprint_back_, footprint_half_width_,
      dwa_corridor_relax_enter_dist_, dwa_corridor_relax_exit_dist_, dwa_path_deviation_hard_cap_,
      footprint_safety_margin_, std::min(cost::kReplanTrigger, safe_cost_threshold_));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      costmap_.resize(width_ * height_, 1.0);
    }

    setupTimers();

    RCLCPP_INFO(
      get_logger(),
      "PRMNode (v10 - plan global unique, replan de routine desactivable) initialise: "
      "carte %dx%d, resolution=%.3f (map_frame='%s', base_frame='%s')",
      width_, height_, resolution_, map_frame_.c_str(), base_frame_.c_str());
  }

private:
  // ---------------------------------------------------------------------
  // Initialisation
  // ---------------------------------------------------------------------
  void declareParameters()
  {
    declare_parameter("map_width", 400);
    declare_parameter("map_height", 400);
    declare_parameter("resolution", 0.05);
    declare_parameter("inflation_radius", 1.2);

    declare_parameter("max_linear_vel", 0.8);
    declare_parameter("max_angular_vel", 1.0);
    declare_parameter("max_linear_accel", 0.7);
    declare_parameter("max_angular_accel", 2.0);
    declare_parameter("lookahead_dist", 1.0);
    declare_parameter("min_lookahead_dist", 0.8);
    declare_parameter("lookahead_curvature_full_reduction", 1.2);
    declare_parameter("local_plan_dt", 0.1);
    declare_parameter("local_plan_steps", 30);
    declare_parameter("safety_buffer", 0.95);

    declare_parameter("footprint_front", 0.80);
    declare_parameter("footprint_back", 0.60);
    declare_parameter("footprint_half_width", 0.46);
    declare_parameter("footprint_safety_margin", 0.12);
    declare_parameter("obstacle_grid_cell_size", 0.3);

    declare_parameter("dwa_v_step", 0.05);
    declare_parameter("dwa_w_step", 0.1);
    declare_parameter("dwa_heading_weight", 1.0);
    declare_parameter("dwa_goal_weight", 1.2);
    declare_parameter("dwa_velocity_weight", 2.0);
    declare_parameter("dwa_obstacle_soft_weight", 1.0);
    declare_parameter("dwa_path_weight", 1.5);
    declare_parameter("dwa_path_corridor_slack", 0.25);
    declare_parameter("dwa_corridor_relax_enter_dist", 0.6);
    declare_parameter("dwa_corridor_relax_exit_dist", 1.2);
    declare_parameter("dwa_path_deviation_hard_cap", 1.5);

    declare_parameter("prm_clearance_weight", 0.6);

    declare_parameter("scan_stride", 3);
    declare_parameter("prm_solve_time", 3.0);
    declare_parameter("path_check_step", 0.15);
    declare_parameter("invalid_path_confirm_count", 6);

    declare_parameter("stuck_timeout", 8.0);
    declare_parameter("stuck_progress_epsilon", 0.03);
    declare_parameter("recovery_rotate_speed", 0.5);
    declare_parameter("recovery_duration", 2.0);
    declare_parameter("recovery_backup_speed", 0.15);
    declare_parameter("recovery_backup_ratio", 0.4);
    declare_parameter("goal_tolerance_relaxed", 0.25);
    declare_parameter("max_recovery_attempts", 3);
    declare_parameter("safety_stop_recovery_timeout", 2.5);

    declare_parameter("local_path_color_r", 0.0);
    declare_parameter("local_path_color_g", 0.4);
    declare_parameter("local_path_color_b", 1.0);
    declare_parameter("local_path_color_a", 0.9);

    declare_parameter("map_frame", "map");
    declare_parameter("base_frame", "base_footprint");

    declare_parameter("goal_tolerance", 0.12);
    declare_parameter("final_approach_radius", 0.6);
    declare_parameter("dwa_final_approach_heading_scale", 0.2);
    declare_parameter("dwa_final_approach_goal_scale", 1.5);

    declare_parameter("turn_in_place_enter_threshold", 1.8);
    declare_parameter("turn_in_place_exit_threshold", 0.35);
    declare_parameter("turn_in_place_kp", 1.2);

    declare_parameter("control_period", 0.05);

    declare_parameter("map_topic", "map");
    declare_parameter("unknown_cost_is_traversable", true);

    declare_parameter("enable_periodic_replan", false);
    declare_parameter("enable_replan_on_blocked_path", false);
  }

  void readParameters()
  {
    width_ = get_parameter("map_width").as_int();
    height_ = get_parameter("map_height").as_int();
    resolution_ = get_parameter("resolution").as_double();
    inflation_radius_ = get_parameter("inflation_radius").as_double();

    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = get_parameter("max_angular_vel").as_double();
    max_linear_accel_ = get_parameter("max_linear_accel").as_double();
    max_angular_accel_ = get_parameter("max_angular_accel").as_double();
    lookahead_dist_ = get_parameter("lookahead_dist").as_double();
    min_lookahead_dist_ = get_parameter("min_lookahead_dist").as_double();
    lookahead_curvature_full_reduction_ = get_parameter("lookahead_curvature_full_reduction").as_double();
    local_plan_dt_ = get_parameter("local_plan_dt").as_double();
    local_plan_steps_ = get_parameter("local_plan_steps").as_int();
    safety_buffer_ = get_parameter("safety_buffer").as_double();

    footprint_front_ = get_parameter("footprint_front").as_double();
    footprint_back_ = get_parameter("footprint_back").as_double();
    footprint_half_width_ = get_parameter("footprint_half_width").as_double();
    footprint_safety_margin_ = get_parameter("footprint_safety_margin").as_double();
    obstacle_grid_cell_size_ = get_parameter("obstacle_grid_cell_size").as_double();


    {
      const double footprint_half_extent =
        std::max({footprint_front_, footprint_back_, footprint_half_width_});
      const double min_required_safety_buffer =
        footprint_half_extent + footprint_safety_margin_ + 0.05;

      if (safety_buffer_ < min_required_safety_buffer) {
        RCLCPP_WARN(
          get_logger(),
          "safety_buffer (%.2f m) insuffisant pour l'empreinte du robot "
          "(half_width=%.2f, front=%.2f, back=%.2f, marge=%.2f m) : le PRM pourrait "
          "planifier a travers des corridors trop etroits que le DWA rejettera "
          "ensuite (arret de securite / but abandonne). Relevement automatique a "
          "%.2f m -- ajustez le parametre 'safety_buffer' pour eviter cet "
          "avertissement.",
          safety_buffer_, footprint_half_width_, footprint_front_, footprint_back_,
          footprint_safety_margin_, min_required_safety_buffer);
        safety_buffer_ = min_required_safety_buffer;
      }


      const double min_required_inflation_radius = 2.0 * safety_buffer_;
      if (inflation_radius_ < min_required_inflation_radius) {
        RCLCPP_WARN(
          get_logger(),
          "inflation_radius (%.2f m) insuffisant par rapport au safety_buffer effectif "
          "(%.2f m) : le seuil de cout resultant tomberait sous le cout 'libre' de base "
          "(kFreeSpace=%.1f), rendant TOUTE la carte invalide pour le PRM (y compris les "
          "zones degagees). Relevement automatique de inflation_radius a %.2f m -- "
          "ajustez le parametre 'inflation_radius' pour eviter cet avertissement.",
          inflation_radius_, safety_buffer_, cost::kFreeSpace, min_required_inflation_radius);
        inflation_radius_ = min_required_inflation_radius;
      }
    }

    dwa_v_step_ = get_parameter("dwa_v_step").as_double();
    dwa_w_step_ = get_parameter("dwa_w_step").as_double();
    dwa_heading_weight_ = get_parameter("dwa_heading_weight").as_double();
    dwa_goal_weight_ = get_parameter("dwa_goal_weight").as_double();
    dwa_velocity_weight_ = get_parameter("dwa_velocity_weight").as_double();
    dwa_obstacle_soft_weight_ = get_parameter("dwa_obstacle_soft_weight").as_double();
    dwa_path_weight_ = get_parameter("dwa_path_weight").as_double();
    dwa_path_corridor_slack_ = get_parameter("dwa_path_corridor_slack").as_double();
    dwa_corridor_relax_enter_dist_ = get_parameter("dwa_corridor_relax_enter_dist").as_double();
    dwa_corridor_relax_exit_dist_ = get_parameter("dwa_corridor_relax_exit_dist").as_double();
    dwa_path_deviation_hard_cap_ = get_parameter("dwa_path_deviation_hard_cap").as_double();

    prm_clearance_weight_ = get_parameter("prm_clearance_weight").as_double();

    scan_stride_ = std::max(1, static_cast<int>(get_parameter("scan_stride").as_int()));
    prm_solve_time_ = get_parameter("prm_solve_time").as_double();
    path_check_step_ = get_parameter("path_check_step").as_double();
    invalid_path_confirm_count_ = std::max(1, static_cast<int>(get_parameter("invalid_path_confirm_count").as_int()));

    stuck_timeout_ = get_parameter("stuck_timeout").as_double();
    stuck_progress_epsilon_ = get_parameter("stuck_progress_epsilon").as_double();
    recovery_rotate_speed_ = get_parameter("recovery_rotate_speed").as_double();
    recovery_duration_ = get_parameter("recovery_duration").as_double();
    recovery_backup_speed_ = get_parameter("recovery_backup_speed").as_double();
    recovery_backup_ratio_ = std::clamp(get_parameter("recovery_backup_ratio").as_double(), 0.0, 0.9);
    goal_tolerance_relaxed_ = get_parameter("goal_tolerance_relaxed").as_double();
    max_recovery_attempts_ = get_parameter("max_recovery_attempts").as_int();
    safety_stop_recovery_timeout_ = get_parameter("safety_stop_recovery_timeout").as_double();

    local_path_color_r_ = get_parameter("local_path_color_r").as_double();
    local_path_color_g_ = get_parameter("local_path_color_g").as_double();
    local_path_color_b_ = get_parameter("local_path_color_b").as_double();
    local_path_color_a_ = get_parameter("local_path_color_a").as_double();

    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    final_approach_radius_ = get_parameter("final_approach_radius").as_double();
    dwa_final_approach_heading_scale_ = get_parameter("dwa_final_approach_heading_scale").as_double();
    dwa_final_approach_goal_scale_ = get_parameter("dwa_final_approach_goal_scale").as_double();

    turn_enter_threshold_ = get_parameter("turn_in_place_enter_threshold").as_double();
    turn_exit_threshold_ = get_parameter("turn_in_place_exit_threshold").as_double();
    turn_kp_ = get_parameter("turn_in_place_kp").as_double();

    control_period_ = get_parameter("control_period").as_double();

    map_topic_ = get_parameter("map_topic").as_string();
    unknown_cost_is_traversable_ = get_parameter("unknown_cost_is_traversable").as_bool();
    enable_periodic_replan_ = get_parameter("enable_periodic_replan").as_bool();
    enable_replan_on_blocked_path_ = get_parameter("enable_replan_on_blocked_path").as_bool();

    safe_cost_threshold_ = computeSafeCostThreshold(safety_buffer_, inflation_radius_);

    if (safe_cost_threshold_ <= cost::kFreeSpace) {
      const double capped_safety_buffer = inflation_radius_ * (1.0 - (cost::kFreeSpace + 2.0) / 15.0);
      RCLCPP_ERROR(
        get_logger(),
        "Seuil de cout derive (%.2f) <= cout 'libre' de base (%.1f) : la carte entiere "
        "serait invalide pour le PRM. Reduction forcee de safety_buffer effectif a "
        "%.2f m (inflation_radius=%.2f m inchange) pour restaurer un seuil exploitable. "
        "Augmentez 'inflation_radius' si vous avez besoin d'un safety_buffer plus grand.",
        safe_cost_threshold_, cost::kFreeSpace, capped_safety_buffer, inflation_radius_);
      safety_buffer_ = std::max(0.1, capped_safety_buffer);
      safe_cost_threshold_ = computeSafeCostThreshold(safety_buffer_, inflation_radius_);
    }

    RCLCPP_INFO(
      get_logger(), "Seuil de cout costmap 'libre' derive: %.2f (safety_buffer=%.2f, inflation_radius=%.2f)",
      safe_cost_threshold_, safety_buffer_, inflation_radius_);
  }

  void setupPublishers()
  {
    path_pub_ = create_publisher<nav_msgs::msg::Path>("prm_path", 10);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("local_path", 10);
    local_path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("local_path_marker", 10);
    start_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("start_pose", 10);
    goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("prm_costmap", 10);
    waypoints_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("prm_waypoints", 10);
    dynamic_obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("dynamic_obstacles", 10);
    robot_footprint_pub_ = create_publisher<visualization_msgs::msg::Marker>("robot_footprint", 10);
    goal_arrow_pub_ = create_publisher<visualization_msgs::msg::Marker>("goal_arrow", 10);
    lookahead_target_pub_ = create_publisher<visualization_msgs::msg::Marker>("lookahead_target", 10);
    status_text_pub_ = create_publisher<visualization_msgs::msg::Marker>("nav_status_text", 10);
  }

  void setupSubscribers()
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PRMNode::odomCallback, this, std::placeholders::_1));

    const auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", scan_qos, std::bind(&PRMNode::scanCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10, std::bind(&PRMNode::goalCallback, this, std::placeholders::_1));

    // Carte statique (nav2_map_server) : publiee en QoS transient_local/reliable, donc
    // recue meme si ce noeud demarre apres le map_server (latched-like behavior).
    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos, std::bind(&PRMNode::mapCallback, this, std::placeholders::_1));
  }

  void setupTimers()
  {
    const auto control_period_ms =
      std::chrono::milliseconds(static_cast<int>(control_period_ * 1000.0));

    visualization_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publishVisualizations();});

    control_timer_ = create_wall_timer(
      control_period_ms,
      [this]() {
        if (!odom_received_ || !goal_received_) {
          return;
        }
        pollPlanningResult();

        if (path_indices_.empty() && !planning_in_progress_) {

          requestReplan(false);
          invalid_path_streak_ = 0;
        } else if (enable_replan_on_blocked_path_ &&
          !path_indices_.empty() && costmap_changed_ && !planning_in_progress_)
        {

          if (!isPathValid()) {
            ++invalid_path_streak_;
          } else {
            invalid_path_streak_ = 0;
          }

          if (invalid_path_streak_ >= invalid_path_confirm_count_) {
            static rclcpp::Time last_plan_time = get_clock()->now();
            if ((get_clock()->now() - last_plan_time).seconds() > 1.5) {
              RCLCPP_WARN(
                get_logger(),
                "Chemin invalide confirme sur %d controles consecutifs: replanification PRM.",
                invalid_path_streak_);
              requestReplan(false);
              last_plan_time = get_clock()->now();
              invalid_path_streak_ = 0;
            }
          }
        }

        followPath();
      });

    replan_timer_ = create_wall_timer(
      std::chrono::milliseconds(4000),
      [this]() {

        if (!enable_periodic_replan_) {
          return;
        }
        if (!odom_received_ || !goal_received_ || path_indices_.empty() || planning_in_progress_) {
          return;
        }
        const double goal_distance = std::hypot(current_x_ - goal_x_, current_y_ - goal_y_);
        if (goal_distance >= 0.3) {
          requestReplan(true);
        }
      });
  }

  // ---------------------------------------------------------------------
  // Etat / validite
  // ---------------------------------------------------------------------
  bool isPathValid()
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    for (size_t i = 0; i + 1 < path_indices_.size(); ++i) {
      double x1, y1, x2, y2;
      indexToCoords(path_indices_[i], width_, resolution_, offset_x_, offset_y_, x1, y1);
      indexToCoords(path_indices_[i + 1], width_, resolution_, offset_x_, offset_y_, x2, y2);
      const double dist = std::hypot(x2 - x1, y2 - y1);
      const int steps = std::max(1, static_cast<int>(dist / path_check_step_));
      for (int s = 0; s <= steps; ++s) {
        const double t = static_cast<double>(s) / steps;
        const double x = x1 + t * (x2 - x1);
        const double y = y1 + t * (y2 - y1);
        const int idx = coordsToIndex(x, y, width_, resolution_, offset_x_, offset_y_);
        if (idx < 0 || idx >= static_cast<int>(costmap_.size()) || costmap_[idx] >= cost::kReplanTrigger) {
          return false;
        }
      }
    }
    return true;
  }

  // ---------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0) {
      RCLCPP_WARN(get_logger(), "Carte recue vide (dimensions nulles), ignoree.");
      return;
    }


    const bool same_dims =
      map_received_ &&
      static_cast<int>(msg->info.width) == width_ &&
      static_cast<int>(msg->info.height) == height_;
    if (same_dims && msg->data == last_map_raw_data_) {
      return;
    }
    last_map_raw_data_ = msg->data;

    std::lock_guard<std::mutex> lock(costmap_mutex_);

    width_ = static_cast<int>(msg->info.width);
    height_ = static_cast<int>(msg->info.height);
    resolution_ = msg->info.resolution;
    offset_x_ = msg->info.origin.position.x;
    offset_y_ = msg->info.origin.position.y;

    static_costmap_.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), cost::kFreeSpace);
    for (size_t i = 0; i < static_costmap_.size(); ++i) {
      const int8_t occ = msg->data[i];
      if (occ < 0) {

        static_costmap_[i] = unknown_cost_is_traversable_ ? cost::kTraversable : cost::kLethal;
      } else if (occ >= 65) {
        static_costmap_[i] = cost::kLethal;
      } else {
        static_costmap_[i] = cost::kFreeSpace;
      }
    }

    for (size_t idx = 0; idx < static_costmap_.size(); ++idx) {
      if (static_costmap_[idx] >= cost::kLethal) {
        inflateStatic(static_cast<int>(idx));
      }
    }

    costmap_ = static_costmap_;
    map_received_ = true;
    costmap_changed_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Carte statique integree: %dx%d @ %.3f m/cell, origine=(%.2f, %.2f)",
      width_, height_, resolution_, offset_x_, offset_y_);
  }

  void inflateStatic(int obs_idx)
  {
    const int obs_x = obs_idx % width_;
    const int obs_y = obs_idx / width_;
    const int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        const int new_x = obs_x + dx;
        const int new_y = obs_y + dy;
        if (new_x < 0 || new_x >= width_ || new_y < 0 || new_y >= height_) {
          continue;
        }
        const int idx = new_y * width_ + new_x;
        const double distance = resolution_ * std::hypot(dx, dy);
        if (distance <= inflation_radius_ && static_costmap_[idx] < cost::kLethal) {
          const double inflated_cost = 15.0 * (1.0 - distance / inflation_radius_);
          static_costmap_[idx] = std::max(static_costmap_[idx], std::max(1.0, inflated_cost));
        }
      }
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped map_pose;
    try {
      tf_buffer_->transform(odom_pose, map_pose, map_frame_, tf2::durationFromSec(2.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Echec de transformation: %s", ex.what());
      return;
    }

    current_x_ = map_pose.pose.position.x;
    current_y_ = map_pose.pose.position.y;

    tf2::Quaternion q(
      map_pose.pose.orientation.x, map_pose.pose.orientation.y,
      map_pose.pose.orientation.z, map_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
    current_yaw_ = normalizeAngle(current_yaw_);

    if (!odom_received_) {
      start_x_ = current_x_;
      start_y_ = current_y_;

      if (!map_received_) {
        offset_x_ = start_x_ - (width_ * resolution_ / 2.0);
        offset_y_ = start_y_ - (height_ * resolution_ / 2.0);
      }
      odom_received_ = true;
      RCLCPP_INFO(
        get_logger(), "Position initiale dans %s: x=%.3f, y=%.3f",
        map_frame_.c_str(), start_x_, start_y_);
    }
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!odom_received_) {
      RCLCPP_WARN(get_logger(), "Goal rejete: aucune odometrie recue, validation impossible.");
      return;
    }
    if (msg->header.frame_id != map_frame_) {
      RCLCPP_WARN(
        get_logger(), "Goal recu dans un frame invalide: %s (attendu: %s)",
        msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    if (!map_received_) {
      RCLCPP_WARN(
        get_logger(),
        "Aucune carte statique recue sur le topic '%s': planification basee uniquement sur "
        "la fenetre glissante autour du depart. Les obstacles non encore vus par le LiDAR "
        "ne seront pas evites tant que la carte n'est pas chargee.",
        map_topic_.c_str());
    }

    const double new_goal_x = msg->pose.position.x;
    const double new_goal_y = msg->pose.position.y;

    if (goal_received_) {
      const double dist_to_current_goal = std::hypot(new_goal_x - goal_x_, new_goal_y - goal_y_);
      if (dist_to_current_goal < 0.05) {
        RCLCPP_DEBUG(get_logger(), "Goal duplique ignore (%.3f m)", dist_to_current_goal);
        return;
      }
    }

    std::lock_guard<std::mutex> lock(costmap_mutex_);
    const int goal_index = coordsToIndex(new_goal_x, new_goal_y, width_, resolution_, offset_x_, offset_y_);
    if (goal_index >= 0 && goal_index < static_cast<int>(costmap_.size()) &&
      costmap_[goal_index] < cost::kGoalAcceptable)
    {
      goal_x_ = new_goal_x;
      goal_y_ = new_goal_y;
      goal_received_ = true;
      path_indices_.clear();
      current_path_length_ = std::numeric_limits<double>::max();
      costmap_changed_ = true;
      is_turning_ = false;
      path_cursor_idx_ = 0;
      pending_plan_.reset();
      planning_in_progress_ = false;
      pending_force_full_replace_ = false;
      best_goal_distance_seen_ = std::numeric_limits<double>::max();
      last_progress_time_ = get_clock()->now();
      recovery_active_ = false;
      recovery_attempts_ = 0;
      safety_stop_active_ = false;
      invalid_path_streak_ = 0;
      RCLCPP_INFO(get_logger(), "Nouveau but dans %s: x=%.3f, y=%.3f", map_frame_.c_str(), goal_x_, goal_y_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "But (%.3f, %.3f) invalide: obstacle ou hors de la fenetre %dx%d "
        "(ajustez map_width/map_height/resolution, ou verifiez le chargement de la carte)",
        new_goal_x, new_goal_y, width_, height_);
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!warned_frame_mismatch_ && msg->header.frame_id != base_frame_) {
      RCLCPP_WARN(
        get_logger(),
        "Frame du scan '%s' != base_frame configure '%s' (non bloquant).",
        msg->header.frame_id.c_str(), base_frame_.c_str());
      warned_frame_mismatch_ = true;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.5));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "Echec TF %s -> %s: %s", msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return;
    }

    std::vector<std::pair<double, double>> new_safety_obstacles;
    std::vector<std::pair<double, double>> new_viz_obstacles;
    new_safety_obstacles.reserve(msg->ranges.size());
    new_viz_obstacles.reserve(msg->ranges.size() / scan_stride_ + 1);

    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);

      costmap_ = map_received_
        ? static_costmap_
        : std::vector<double>(static_cast<size_t>(width_) * static_cast<size_t>(height_), 1.0);

      for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const double range = msg->ranges[i];
        if (range <= msg->range_min || range >= msg->range_max) {
          continue;
        }
        const double angle = msg->angle_min + i * msg->angle_increment;

        geometry_msgs::msg::Point laser_point;
        laser_point.x = range * std::cos(angle);
        laser_point.y = range * std::sin(angle);

        geometry_msgs::msg::Point map_point;
        tf2::doTransform(laser_point, map_point, transform);

        const int obs_idx = coordsToIndex(map_point.x, map_point.y, width_, resolution_, offset_x_, offset_y_);
        if (obs_idx >= 0 && obs_idx < static_cast<int>(costmap_.size()) && costmap_[obs_idx] < cost::kLethal) {
          costmap_[obs_idx] = cost::kLethal;
          inflateObstacle(obs_idx);
        }

        new_safety_obstacles.emplace_back(map_point.x, map_point.y);
        if (i % scan_stride_ == 0) {
          new_viz_obstacles.emplace_back(map_point.x, map_point.y);
        }
      }

      costmap_changed_ = true;
    }

    dynamic_obstacles_ = std::move(new_safety_obstacles);
    dynamic_obstacles_viz_ = std::move(new_viz_obstacles);
  }


  void inflateObstacle(int obs_idx)
  {
    const int obs_x = obs_idx % width_;
    const int obs_y = obs_idx / width_;
    const int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        const int new_x = obs_x + dx;
        const int new_y = obs_y + dy;
        if (new_x < 0 || new_x >= width_ || new_y < 0 || new_y >= height_) {
          continue;
        }
        const int idx = new_y * width_ + new_x;
        const double distance = resolution_ * std::hypot(dx, dy);
        if (distance <= inflation_radius_ && costmap_[idx] < cost::kLethal) {
          const double inflated_cost = 15.0 * (1.0 - distance / inflation_radius_);
          costmap_[idx] = std::max(costmap_[idx], std::max(1.0, inflated_cost));
        }
      }
    }
  }

  // ---------------------------------------------------------------------
  // Planification globale (asynchrone)
  // ---------------------------------------------------------------------
  struct PendingPlan
  {
    std::future<std::vector<int>> future;
    bool replace_if_shorter_only = false;
  };

  void relaxCostmapBubble(std::vector<double> & costmap, double cx, double cy, double radius) const
  {
    const int cells = std::max(1, static_cast<int>(std::ceil(radius / resolution_)));
    const int center_x = static_cast<int>(std::round((cx - offset_x_) / resolution_));
    const int center_y = static_cast<int>(std::round((cy - offset_y_) / resolution_));

    for (int dx = -cells; dx <= cells; ++dx) {
      for (int dy = -cells; dy <= cells; ++dy) {
        if (resolution_ * std::hypot(dx, dy) > radius) {
          continue;
        }
        const int x = center_x + dx;
        const int y = center_y + dy;
        if (x < 0 || x >= width_ || y < 0 || y >= height_) {
          continue;
        }
        const int idx = y * width_ + x;
        if (idx >= 0 && idx < static_cast<int>(costmap.size()) && costmap[idx] < cost::kLethal) {
          costmap[idx] = cost::kFreeSpace;
        }
      }
    }
  }

  bool requestReplan(bool replace_if_shorter_only)
  {
    if (planning_in_progress_ || !odom_received_ || !goal_received_) {
      return false;
    }

    if (!map_received_) {
      offset_x_ = start_x_ - (width_ * resolution_ / 2.0);
      offset_y_ = start_y_ - (height_ * resolution_ / 2.0);
    }

    std::vector<double> costmap_snapshot;
    int start_index, goal_index;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      costmap_snapshot = costmap_;
      start_index = coordsToIndex(current_x_, current_y_, width_, resolution_, offset_x_, offset_y_);
      goal_index = coordsToIndex(goal_x_, goal_y_, width_, resolution_, offset_x_, offset_y_);
    }


    const double relax_radius =
      std::max({footprint_front_, footprint_back_, footprint_half_width_}) + 0.05;
    relaxCostmapBubble(costmap_snapshot, current_x_, current_y_, relax_radius);
    relaxCostmapBubble(costmap_snapshot, goal_x_, goal_y_, relax_radius);

    if (start_index < 0 || start_index >= width_ * height_ ||
      costmap_snapshot[start_index] >= cost::kStartAcceptable)
    {
      RCLCPP_WARN(get_logger(), "Depart invalide (idx=%d), replan annule.", start_index);
      return false;
    }
    if (goal_index < 0 || goal_index >= width_ * height_ ||
      costmap_snapshot[goal_index] >= cost::kGoalAcceptable)
    {
      RCLCPP_WARN(get_logger(), "But invalide (idx=%d), replan annule.", goal_index);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Lancement de la planification PRM (asynchrone)...");

    planning_in_progress_ = true;
    const int w = width_;
    const int h = height_;
    const double res = resolution_;
    const double ox = offset_x_;
    const double oy = offset_y_;
    const double solve_time = prm_solve_time_;
    const double sct = safe_cost_threshold_;
    const double clearance_weight = prm_clearance_weight_;

    auto fut = std::async(
      std::launch::async,
      [start_index, goal_index, w, h, costmap_snapshot, res, ox, oy, solve_time, sct,
        clearance_weight]() mutable {
        const auto raw = planPRM(
          start_index, goal_index, w, h, costmap_snapshot, res, ox, oy, solve_time, sct,
          clearance_weight);
        return smoothPath(raw, w, h, res, ox, oy, costmap_snapshot, sct);
      });

    pending_plan_ = PendingPlan{std::move(fut), replace_if_shorter_only};
    return true;
  }

  void pollPlanningResult()
  {
    if (!pending_plan_.has_value()) {
      return;
    }
    if (pending_plan_->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return;
    }

    auto new_path = pending_plan_->future.get();

    const bool force_full_replace = pending_force_full_replace_;
    pending_force_full_replace_ = false;
    const bool replace_if_shorter_only = pending_plan_->replace_if_shorter_only && !force_full_replace;
    pending_plan_.reset();
    planning_in_progress_ = false;

    if (new_path.empty()) {
      RCLCPP_WARN(get_logger(), "Planification asynchrone: aucun chemin valide trouve.");
      if (!replace_if_shorter_only) {
        current_path_length_ = std::numeric_limits<double>::max();
      }
      return;
    }

    const double new_length = computePathLength(new_path, width_, resolution_, offset_x_, offset_y_);

    if (!replace_if_shorter_only) {
      path_indices_ = std::move(new_path);
      current_path_length_ = new_length;
      path_cursor_idx_ = 0;
      costmap_changed_ = false;
      RCLCPP_INFO(
        get_logger(), "Nouveau chemin PRM: %zu points, longueur=%.2f m", path_indices_.size(), new_length);
    } else if (new_length < current_path_length_ * 0.9) {

      if (std::isfinite(current_path_length_)) {
        RCLCPP_INFO(
          get_logger(), "Meilleur chemin trouve: %.2f m -> %.2f m", current_path_length_, new_length);
      } else {
        RCLCPP_INFO(get_logger(), "Chemin remplace (aucune reference precedente) -> %.2f m", new_length);
      }
      path_indices_ = std::move(new_path);
      current_path_length_ = new_length;
      path_cursor_idx_ = 0;
      costmap_changed_ = false;
    }
  }

  // ---------------------------------------------------------------------
  // Suivi de trajectoire (pure pursuit + DWA)
  // ---------------------------------------------------------------------
  size_t findNearestPathIndex() const
  {
    constexpr size_t kSearchWindow = 200;
    const size_t end = std::min(path_indices_.size(), path_cursor_idx_ + kSearchWindow);
    size_t best_idx = path_cursor_idx_;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = path_cursor_idx_; i < end; ++i) {
      double px, py;
      indexToCoords(path_indices_[i], width_, resolution_, offset_x_, offset_y_, px, py);
      const double d = std::hypot(px - current_x_, py - current_y_);
      if (d < best_dist) {
        best_dist = d;
        best_idx = i;
      }
    }
    return best_idx;
  }

  double computePathCurvatureAhead(size_t start_idx, double max_arc_length) const
  {
    if (start_idx + 1 >= path_indices_.size()) {
      return 0.0;
    }
    double accumulated = 0.0;
    double total_turn = 0.0;
    bool have_prev_heading = false;
    double prev_heading = 0.0;
    size_t i = start_idx;
    while (i + 1 < path_indices_.size() && accumulated < max_arc_length) {
      double x1, y1, x2, y2;
      indexToCoords(path_indices_[i], width_, resolution_, offset_x_, offset_y_, x1, y1);
      indexToCoords(path_indices_[i + 1], width_, resolution_, offset_x_, offset_y_, x2, y2);
      const double seg_len = std::hypot(x2 - x1, y2 - y1);
      if (seg_len > 1e-6) {
        const double heading = std::atan2(y2 - y1, x2 - x1);
        if (have_prev_heading) {
          total_turn += std::abs(normalizeAngle(heading - prev_heading));
        }
        prev_heading = heading;
        have_prev_heading = true;
      }
      accumulated += seg_len;
      ++i;
    }
    return total_turn;
  }

  double computeAdaptiveLookaheadDist(size_t nearest_idx) const
  {
    const double turn_angle = computePathCurvatureAhead(nearest_idx, lookahead_dist_);
    const double turn_ratio = std::clamp(turn_angle / lookahead_curvature_full_reduction_, 0.0, 1.0);
    return std::max(
      min_lookahead_dist_,
      lookahead_dist_ * (1.0 - turn_ratio * (1.0 - min_lookahead_dist_ / lookahead_dist_)));
  }

  std::pair<double, double> computeLookaheadTarget(size_t nearest_idx, double lookahead_distance)
  {
    double px, py;
    indexToCoords(path_indices_[nearest_idx], width_, resolution_, offset_x_, offset_y_, px, py);

    double accumulated = 0.0;
    size_t i = nearest_idx;
    while (i + 1 < path_indices_.size() && accumulated < lookahead_distance) {
      double x1, y1, x2, y2;
      indexToCoords(path_indices_[i], width_, resolution_, offset_x_, offset_y_, x1, y1);
      indexToCoords(path_indices_[i + 1], width_, resolution_, offset_x_, offset_y_, x2, y2);
      accumulated += std::hypot(x2 - x1, y2 - y1);
      px = x2;
      py = y2;
      ++i;
    }
    return {px, py};
  }


  bool triggerRecovery()
  {
    const double goal_distance = std::hypot(current_x_ - goal_x_, current_y_ - goal_y_);

    if (goal_distance < goal_tolerance_relaxed_) {
      RCLCPP_WARN(
        get_logger(),
        "Robot bloque a %.3f m du but: but considere atteint (tolerance relachee) pour "
        "eviter une recherche infinie.",
        goal_distance);
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      goal_received_ = false;
      path_indices_.clear();
      path_cursor_idx_ = 0;
      current_path_length_ = std::numeric_limits<double>::max();
      is_turning_ = false;
      local_traj_valid_ = false;
      last_cmd_v_ = 0.0;
      last_cmd_w_ = 0.0;
      recovery_attempts_ = 0;
      return false;
    }

    ++recovery_attempts_;
    if (recovery_attempts_ > max_recovery_attempts_) {
      RCLCPP_ERROR(
        get_logger(),
        "Robot bloque a %.3f m du but apres %d tentative(s) de recuperation. Abandon du "
        "but -- envoyez un nouveau but pour reessayer.",
        goal_distance, max_recovery_attempts_);
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      goal_received_ = false;
      path_indices_.clear();
      path_cursor_idx_ = 0;
      current_path_length_ = std::numeric_limits<double>::max();
      is_turning_ = false;
      local_traj_valid_ = false;
      last_cmd_v_ = 0.0;
      last_cmd_w_ = 0.0;
      recovery_attempts_ = 0;
      return false;
    }

    RCLCPP_WARN(
      get_logger(), "Declenchement de la recuperation #%d (but a %.3f m).",
      recovery_attempts_, goal_distance);
    recovery_active_ = true;
    recovery_direction_ = (recovery_attempts_ % 2 == 0) ? -1.0 : 1.0;
    recovery_end_time_ = get_clock()->now() + rclcpp::Duration::from_seconds(recovery_duration_);
    return true;
  }

  bool handleStuckRecovery()
  {
    const rclcpp::Time now = get_clock()->now();
    const double goal_distance = std::hypot(current_x_ - goal_x_, current_y_ - goal_y_);

    if (goal_distance < best_goal_distance_seen_ - stuck_progress_epsilon_) {
      best_goal_distance_seen_ = goal_distance;
      last_progress_time_ = now;
      if (recovery_active_) {
        recovery_active_ = false;
        recovery_attempts_ = 0;
        RCLCPP_INFO(get_logger(), "Progression detectee vers le but, sortie du mode recuperation.");
      }
      return false;
    }

    if (recovery_active_) {
      if (now < recovery_end_time_) {

        const double elapsed = recovery_duration_ - (recovery_end_time_ - now).seconds();
        const double backup_phase_end = recovery_duration_ * recovery_backup_ratio_;

        geometry_msgs::msg::Twist cmd;
        if (elapsed < backup_phase_end && recovery_backup_speed_ > 1e-6) {
          ObstacleGrid obstacle_grid(obstacle_grid_cell_size_);
          obstacle_grid.build(dynamic_obstacles_);
          const double clearance_behind = obstacle_grid.distanceToNearest(
            current_x_ - footprint_back_ * std::cos(current_yaw_),
            current_y_ - footprint_back_ * std::sin(current_yaw_),
            safety_buffer_ * 1.5);
          if (clearance_behind > safety_buffer_) {
            cmd.linear.x = -recovery_backup_speed_;
          }
        } else {
          cmd.angular.z = recovery_direction_ * recovery_rotate_speed_;
        }
        cmd_vel_pub_->publish(cmd);
        last_cmd_v_ = cmd.linear.x;
        last_cmd_w_ = cmd.angular.z;
        local_traj_valid_ = false;
        return true;
      }
      recovery_active_ = false;
      last_progress_time_ = now;
      safety_stop_active_ = false;
      RCLCPP_INFO(get_logger(), "Recuperation terminee, replanification complete forcee.");
      path_indices_.clear();
      current_path_length_ = std::numeric_limits<double>::max();
      if (planning_in_progress_) {

        pending_force_full_replace_ = true;
      } else {
        requestReplan(false);
      }
      return true;
    }

    if ((now - last_progress_time_).seconds() > stuck_timeout_) {
      triggerRecovery();
      return true;
    }

    return false;
  }

  void followPath()
  {
    if (path_indices_.empty()) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      last_cmd_v_ = 0.0;
      last_cmd_w_ = 0.0;
      local_traj_valid_ = false;
      return;
    }

    const double goal_distance = std::hypot(current_x_ - goal_x_, current_y_ - goal_y_);
    if (goal_distance < goal_tolerance_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      RCLCPP_INFO(get_logger(), "But atteint (%.2f, %.2f), distance=%.3f m", goal_x_, goal_y_, goal_distance);
      goal_received_ = false;
      path_indices_.clear();
      path_cursor_idx_ = 0;
      current_path_length_ = std::numeric_limits<double>::max();
      is_turning_ = false;
      local_traj_valid_ = false;
      last_cmd_v_ = 0.0;
      last_cmd_w_ = 0.0;
      recovery_attempts_ = 0;
      return;
    }

    if (handleStuckRecovery()) {
      return;
    }

    path_cursor_idx_ = findNearestPathIndex();
    const double adaptive_lookahead = computeAdaptiveLookaheadDist(path_cursor_idx_);
    const auto [target_x, target_y] = computeLookaheadTarget(path_cursor_idx_, adaptive_lookahead);
    target_x_ = target_x;
    target_y_ = target_y;

    const bool in_final_approach = goal_distance < final_approach_radius_;
    if (in_final_approach) {
      target_x_ = goal_x_;
      target_y_ = goal_y_;
      if (is_turning_) {
        is_turning_ = false;
        RCLCPP_INFO(get_logger(), "Approche finale: rotation sur place annulee");
      }
    }

    const double desired_yaw = std::atan2(target_y_ - current_y_, target_x_ - current_x_);
    const double yaw_diff = normalizeAngle(desired_yaw - current_yaw_);

    if (!in_final_approach) {
      if (!is_turning_ && std::abs(yaw_diff) > turn_enter_threshold_) {
        is_turning_ = true;
        RCLCPP_INFO(get_logger(), "Rotation sur place initiee (yaw_diff=%.2f rad)", yaw_diff);
      }

      if (is_turning_) {
        if (std::abs(yaw_diff) < turn_exit_threshold_) {
          is_turning_ = false;
          RCLCPP_INFO(get_logger(), "Rotation sur place terminee");
        } else {
          geometry_msgs::msg::Twist turn_cmd;
          const double w_cmd = std::clamp(turn_kp_ * yaw_diff, -max_angular_vel_, max_angular_vel_);
          turn_cmd.angular.z = w_cmd;
          cmd_vel_pub_->publish(turn_cmd);
          last_cmd_v_ = 0.0;
          last_cmd_w_ = w_cmd;
          local_traj_valid_ = false;
          return;
        }
      }
    }

    std::vector<double> costmap_snapshot;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      costmap_snapshot = costmap_;
    }

    ObstacleGrid obstacle_grid(obstacle_grid_cell_size_);
    obstacle_grid.build(dynamic_obstacles_);

    const double heading_weight_scale = in_final_approach ? dwa_final_approach_heading_scale_ : 1.0;
    const double goal_weight_scale = in_final_approach ? dwa_final_approach_goal_scale_ : 1.0;

    std::vector<std::pair<double, double>> path_ref_points;
    if (!in_final_approach) {
      const size_t start_ref = (path_cursor_idx_ > 5) ? path_cursor_idx_ - 5 : 0;
      const double cutoff_length = adaptive_lookahead * 2.0 + 1.0;

      double relevant_length = 0.0;
      for (size_t i = start_ref; i + 1 < path_indices_.size() && relevant_length < cutoff_length; ++i) {
        double x1, y1, x2, y2;
        indexToCoords(path_indices_[i], width_, resolution_, offset_x_, offset_y_, x1, y1);
        indexToCoords(path_indices_[i + 1], width_, resolution_, offset_x_, offset_y_, x2, y2);
        relevant_length += std::hypot(x2 - x1, y2 - y1);
      }
      const double sample_step = std::clamp(relevant_length / 150.0, 0.05, 0.20);

      double accumulated = 0.0;
      for (size_t i = start_ref; i + 1 < path_indices_.size() && accumulated < cutoff_length; ++i) {
        double x1, y1, x2, y2;
        indexToCoords(path_indices_[i], width_, resolution_, offset_x_, offset_y_, x1, y1);
        indexToCoords(path_indices_[i + 1], width_, resolution_, offset_x_, offset_y_, x2, y2);
        const double seg_len = std::hypot(x2 - x1, y2 - y1);
        const int n_samples = std::max(1, static_cast<int>(seg_len / sample_step));
        for (int s = 0; s <= n_samples; ++s) {
          const double t = static_cast<double>(s) / n_samples;
          path_ref_points.emplace_back(x1 + t * (x2 - x1), y1 + t * (y2 - y1));
        }
        accumulated += seg_len;
      }

      if (path_ref_points.empty()) {
 
        for (size_t i = start_ref; i < path_indices_.size(); ++i) {
          double px, py;
          indexToCoords(path_indices_[i], width_, resolution_, offset_x_, offset_y_, px, py);
          path_ref_points.emplace_back(px, py);
        }
      }
    }

    const auto local_traj = local_planner_->computeBestTrajectory(
      current_x_, current_y_, current_yaw_, target_x_, target_y_,
      costmap_snapshot, width_, height_, offset_x_, offset_y_,
      obstacle_grid, last_cmd_v_, last_cmd_w_, control_period_,
      heading_weight_scale, goal_weight_scale, path_ref_points);

    local_path_ = local_traj;
    local_traj_valid_ = local_traj.valid;

    if (!local_traj.valid) {
      const rclcpp::Time now = get_clock()->now();
      if (!safety_stop_active_) {
        safety_stop_active_ = true;
        safety_stop_since_ = now;
      } else if (!recovery_active_ && (now - safety_stop_since_).seconds() > safety_stop_recovery_timeout_) {
        RCLCPP_WARN(
          get_logger(),
          "Arret de securite prolonge depuis plus de %.1f s (aucune trajectoire locale sure): "
          "declenchement immediat de la recuperation.",
          safety_stop_recovery_timeout_);
        triggerRecovery();
      }
      RCLCPP_WARN(
        get_logger(), "Aucune trajectoire locale sure trouvee, arret de securite (bloque depuis %.1f s).",
        (now - safety_stop_since_).seconds());
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      last_cmd_v_ = 0.0;
      last_cmd_w_ = 0.0;
      costmap_changed_ = true;
      return;
    }
    safety_stop_active_ = false;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = local_traj.linear_vel;
    cmd.angular.z = local_traj.angular_vel;
    cmd_vel_pub_->publish(cmd);
    last_cmd_v_ = local_traj.linear_vel;
    last_cmd_w_ = local_traj.angular_vel;
  }

  // ---------------------------------------------------------------------
  // Visualisation RViz
  // ---------------------------------------------------------------------
  void publishVisualizations()
  {
    const rclcpp::Time now = get_clock()->now();

    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.stamp = now;
    start_pose.header.frame_id = map_frame_;
    start_pose.pose.position.x = start_x_;
    start_pose.pose.position.y = start_y_;
    start_pose.pose.orientation.w = 1.0;
    start_pose_pub_->publish(start_pose);

    if (goal_received_) {
      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.stamp = now;
      goal_pose.header.frame_id = map_frame_;
      goal_pose.pose.position.x = goal_x_;
      goal_pose.pose.position.y = goal_y_;
      goal_pose.pose.orientation.w = 1.0;
      goal_pose_pub_->publish(goal_pose);
    }

    if (!path_indices_.empty()) {
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = now;
      path_msg.header.frame_id = map_frame_;
      for (const int index : path_indices_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        indexToCoords(
          index, width_, resolution_, offset_x_, offset_y_,
          pose.pose.position.x, pose.pose.position.y);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
      }
      path_pub_->publish(path_msg);
    }

    nav_msgs::msg::Path local_path_msg;
    local_path_msg.header.stamp = now;
    local_path_msg.header.frame_id = map_frame_;
    if (local_traj_valid_) {
      for (const auto & [x, y] : local_path_.points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = local_path_msg.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;
        local_path_msg.poses.push_back(pose);
      }
    }
    local_path_pub_->publish(local_path_msg);

    visualization_msgs::msg::Marker local_path_marker;
    local_path_marker.header = local_path_msg.header;
    local_path_marker.ns = "local_path";
    local_path_marker.id = 0;
    local_path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    local_path_marker.action = local_traj_valid_
      ? visualization_msgs::msg::Marker::ADD
      : visualization_msgs::msg::Marker::DELETE;
    local_path_marker.scale.x = 0.05;
    local_path_marker.pose.orientation.w = 1.0;
    local_path_marker.color.r = static_cast<float>(local_path_color_r_);
    local_path_marker.color.g = static_cast<float>(local_path_color_g_);
    local_path_marker.color.b = static_cast<float>(local_path_color_b_);
    local_path_marker.color.a = static_cast<float>(local_path_color_a_);
    local_path_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    if (local_traj_valid_) {
      for (const auto & [x, y] : local_path_.points) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.03;
        local_path_marker.points.push_back(p);
      }
    }
    local_path_marker_pub_->publish(local_path_marker);

    publishCostmap();
    publishWaypointsMarkers();
    publishDynamicObstaclesMarkers();
    publishRobotFootprint();
    publishGoalArrow();
    publishLookaheadTarget();
    publishStatusText();
  }

  void publishDynamicObstaclesMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    const rclcpp::Time now = get_clock()->now();

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int id = 0;
    for (const auto & [ox, oy] : dynamic_obstacles_viz_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = now;
      m.ns = "dynamic_obstacles";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = ox;
      m.pose.position.y = oy;
      m.pose.position.z = 0.1;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = safety_buffer_ * 0.15;
      m.scale.z = 0.2;
      m.color.r = 1.0; m.color.g = 0.3; m.color.b = 0.0; m.color.a = 0.6;
      m.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(m);
    }
    dynamic_obs_pub_->publish(marker_array);
  }

  void publishWaypointsMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    const rclcpp::Time now = get_clock()->now();

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (size_t i = 0; i < path_indices_.size(); ++i) {
      double x, y;
      indexToCoords(path_indices_[i], width_, resolution_, offset_x_, offset_y_, x, y);

      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = now;
      m.ns = "waypoints";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = 0.05;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.08;
      m.lifetime = rclcpp::Duration::from_seconds(1.0);

      if (i == 0) {
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
      } else if (i == path_indices_.size() - 1) {
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
      } else if (i == path_cursor_idx_) {
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.15;
      } else {
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;
      }
      marker_array.markers.push_back(m);
    }
    waypoints_pub_->publish(marker_array);
  }

  void publishCostmap()
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = get_clock()->now();
    grid.header.frame_id = map_frame_;
    grid.info.resolution = resolution_;
    grid.info.width = width_;
    grid.info.height = height_;
    grid.info.origin.position.x = offset_x_;
    grid.info.origin.position.y = offset_y_;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(width_ * height_);
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      for (int i = 0; i < width_ * height_; ++i) {
        const double c = costmap_[i];
        if (c >= cost::kLethal) {
          grid.data[i] = 100;
        } else if (c >= cost::kReplanTrigger) {
          grid.data[i] = 99;
        } else if (c >= 1.0) {
          grid.data[i] = static_cast<int8_t>(c / 10.0 * 98);
        } else {
          grid.data[i] = 0;
        }
      }
    }
    costmap_pub_->publish(grid);
  }

  void publishRobotFootprint()
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = get_clock()->now();
    m.ns = "robot_footprint";
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;

    const double cos_y = std::cos(current_yaw_);
    const double sin_y = std::sin(current_yaw_);
    const std::vector<std::pair<double, double>> corners = {
      {footprint_front_, footprint_half_width_},
      {footprint_front_, -footprint_half_width_},
      {-footprint_back_, -footprint_half_width_},
      {-footprint_back_, footprint_half_width_},
      {footprint_front_, footprint_half_width_},
    };
    for (const auto & [lx, ly] : corners) {
      geometry_msgs::msg::Point p;
      p.x = current_x_ + lx * cos_y - ly * sin_y;
      p.y = current_y_ + lx * sin_y + ly * cos_y;
      p.z = 0.02;
      m.points.push_back(p);
    }
    m.scale.x = 0.02;
    m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 0.8f;
    m.lifetime = rclcpp::Duration::from_seconds(0.2);
    robot_footprint_pub_->publish(m);
  }

  void publishGoalArrow()
  {
    if (!goal_received_) {
      return;
    }
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = get_clock()->now();
    m.ns = "goal_arrow";
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start_pt, end_pt;
    start_pt.x = current_x_; start_pt.y = current_y_; start_pt.z = 0.1;
    end_pt.x = goal_x_; end_pt.y = goal_y_; end_pt.z = 0.1;
    m.points.push_back(start_pt);
    m.points.push_back(end_pt);

    m.scale.x = 0.04; m.scale.y = 0.10; m.scale.z = 0.12;
    m.color.r = 0.9f; m.color.g = 0.0f; m.color.b = 0.9f; m.color.a = 0.6f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    goal_arrow_pub_->publish(m);
  }

  void publishLookaheadTarget()
  {
    if (!goal_received_ || path_indices_.empty()) {
      return;
    }
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = get_clock()->now();
    m.ns = "lookahead_target";
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = target_x_;
    m.pose.position.y = target_y_;
    m.pose.position.z = 0.1;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.18;
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 1.0f; m.color.a = 0.9f;
    m.lifetime = rclcpp::Duration::from_seconds(0.2);
    lookahead_target_pub_->publish(m);
  }

  void publishStatusText()
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = get_clock()->now();
    m.ns = "status_text";
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = current_x_;
    m.pose.position.y = current_y_;
    m.pose.position.z = 0.5;
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.15;

    const double goal_dist = std::hypot(current_x_ - goal_x_, current_y_ - goal_y_);
    std::ostringstream oss;
    if (!goal_received_) {
      oss << "IDLE";
      m.color.r = 0.6f; m.color.g = 0.6f; m.color.b = 0.6f; m.color.a = 1.0f;
    } else if (path_indices_.empty()) {
      oss << (planning_in_progress_ ? "PLANNING..." : "EN ATTENTE");
      m.color.r = 1.0f; m.color.g = 0.8f; m.color.b = 0.0f; m.color.a = 1.0f;
    } else if (is_turning_) {
      oss << "ROTATION SUR PLACE";
      m.color.r = 0.2f; m.color.g = 0.6f; m.color.b = 1.0f; m.color.a = 1.0f;
    } else if (!local_traj_valid_) {
      oss << "ARRET SECURITE";
      m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 1.0f;
    } else {
      oss << std::fixed << std::setprecision(2) << "NAV " << goal_dist << " m";
      m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.4f; m.color.a = 1.0f;
    }
    m.text = oss.str();
    m.lifetime = rclcpp::Duration::from_seconds(0.3);
    status_text_pub_->publish(m);
  }

  // ---------------------------------------------------------------------
  // Publishers / Subscribers
  // ---------------------------------------------------------------------
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_path_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_footprint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_arrow_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr status_text_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr replan_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<DwaLocalPlanner> local_planner_;
  DwaLocalPlanner::Trajectory local_path_;
  bool local_traj_valid_ = false;

  // ---------------------------------------------------------------------
  // Etat interne
  // ---------------------------------------------------------------------
  std::vector<int> path_indices_;
  size_t path_cursor_idx_ = 0;

  std::optional<PendingPlan> pending_plan_;
  bool planning_in_progress_ = false;
  bool pending_force_full_replace_ = false;

  double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
  double target_x_ = 0.0, target_y_ = 0.0;
  double start_x_ = 0.0, start_y_ = 0.0;
  double offset_x_ = 0.0, offset_y_ = 0.0;
  double goal_x_ = 0.0, goal_y_ = 0.0;

  double last_cmd_v_ = 0.0, last_cmd_w_ = 0.0;

  bool odom_received_ = false;
  bool goal_received_ = false;
  bool costmap_changed_ = false;
  bool is_turning_ = false;
  bool warned_frame_mismatch_ = false;

  std::mutex costmap_mutex_;
  std::vector<double> costmap_;
  std::vector<double> static_costmap_;
  bool map_received_ = false;
  std::vector<int8_t> last_map_raw_data_;
  std::string map_topic_ = "map";
  bool unknown_cost_is_traversable_ = true;
  bool enable_periodic_replan_ = false;
  bool enable_replan_on_blocked_path_ = false;
  std::vector<std::pair<double, double>> dynamic_obstacles_;
  std::vector<std::pair<double, double>> dynamic_obstacles_viz_;

  int width_ = 0, height_ = 0;
  double resolution_ = 0.05, inflation_radius_ = 0.75;
  double max_linear_vel_ = 0.4, max_angular_vel_ = 1.0;
  double max_linear_accel_ = 0.5, max_angular_accel_ = 2.0;
  double lookahead_dist_ = 1.0;
  double min_lookahead_dist_ = 0.4;
  double lookahead_curvature_full_reduction_ = 1.2;
  double local_plan_dt_ = 0.1;
  int local_plan_steps_ = 30;
  double safety_buffer_ = 0.65;
  double footprint_front_ = 0.80, footprint_back_ = 0.60, footprint_half_width_ = 0.46;
  double footprint_safety_margin_ = 0.12;
  double obstacle_grid_cell_size_ = 0.3;
  double dwa_v_step_ = 0.05, dwa_w_step_ = 0.1;
  double dwa_heading_weight_ = 1.0, dwa_goal_weight_ = 1.2;
  double dwa_velocity_weight_ = 1.5, dwa_obstacle_soft_weight_ = 1.0;
  double dwa_path_weight_ = 1.5, dwa_path_corridor_slack_ = 0.25;
  double dwa_corridor_relax_enter_dist_ = 0.6, dwa_corridor_relax_exit_dist_ = 1.2;
  double dwa_path_deviation_hard_cap_ = 1.5;
  double prm_clearance_weight_ = 0.6;
  int scan_stride_ = 3;
  double prm_solve_time_ = 3.0;
  double path_check_step_ = 0.15;
  int invalid_path_confirm_count_ = 6;
  int invalid_path_streak_ = 0;
  double goal_tolerance_ = 0.12;
  double final_approach_radius_ = 0.6;
  double dwa_final_approach_heading_scale_ = 0.2;
  double dwa_final_approach_goal_scale_ = 1.5;
  double turn_enter_threshold_ = 1.8, turn_exit_threshold_ = 0.35, turn_kp_ = 1.2;
  double control_period_ = 0.05;
  double safe_cost_threshold_ = cost::kFreeSpace;
  double current_path_length_ = std::numeric_limits<double>::max();
  std::string map_frame_ = "map", base_frame_ = "base_footprint";

  // ---------------------------------------------------------------------
  // Watchdog anti-blocage (recherche infinie pres du but)
  // ---------------------------------------------------------------------
  double stuck_timeout_ = 8.0;
  double stuck_progress_epsilon_ = 0.03;
  double recovery_rotate_speed_ = 0.5;
  double recovery_duration_ = 2.0;
  double recovery_backup_speed_ = 0.15;
  double recovery_backup_ratio_ = 0.4;
  double goal_tolerance_relaxed_ = 0.25;
  int max_recovery_attempts_ = 3;
  double safety_stop_recovery_timeout_ = 2.5;

  double best_goal_distance_seen_ = std::numeric_limits<double>::max();
  rclcpp::Time last_progress_time_;
  bool recovery_active_ = false;
  rclcpp::Time recovery_end_time_;
  double recovery_direction_ = 1.0;
  int recovery_attempts_ = 0;
  bool safety_stop_active_ = false;
  rclcpp::Time safety_stop_since_;

  // ---------------------------------------------------------------------
  // Couleur du marker de chemin local (RViz)
  // ---------------------------------------------------------------------
  double local_path_color_r_ = 0.0, local_path_color_g_ = 0.4;
  double local_path_color_b_ = 1.0, local_path_color_a_ = 0.9;
};

}  // namespace pearlguard_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pearlguard_navigation::PRMNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
