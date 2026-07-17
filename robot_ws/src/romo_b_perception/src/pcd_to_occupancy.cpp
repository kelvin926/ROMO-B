#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace
{
struct GraphPose
{
  double x{};
  double y{};
  double z{};
};

struct ObstaclePoint
{
  pcl::PointXYZ point;
  std::size_t nearest_pose{};
};

std::vector<GraphPose> load_graph_poses(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Cannot open pose graph " + path.string());
  }
  std::vector<GraphPose> poses;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream stream(line);
    std::string record_type;
    stream >> record_type;
    if (record_type != "VERTEX_SE3:QUAT") {
      continue;
    }
    std::size_t id{};
    GraphPose pose;
    if (stream >> id >> pose.x >> pose.y >> pose.z) {
      poses.push_back(pose);
    }
  }
  if (poses.empty()) {
    throw std::runtime_error("No VERTEX_SE3:QUAT records in " + path.string());
  }
  return poses;
}

std::pair<std::size_t, double> nearest_pose(
  const pcl::PointXYZ & point, const std::vector<GraphPose> & poses)
{
  std::size_t best_index = 0;
  double best_squared_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < poses.size(); ++index) {
    const double dx = point.x - poses[index].x;
    const double dy = point.y - poses[index].y;
    const double squared_distance = dx * dx + dy * dy;
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best_index = index;
    }
  }
  return {best_index, std::sqrt(best_squared_distance)};
}

void mark_ray_free(
  int start_x, int start_y, int end_x, int end_y, std::size_t width,
  std::size_t height, std::vector<bool> & free_cells)
{
  int x = start_x;
  int y = start_y;
  const int dx = std::abs(end_x - start_x);
  const int step_x = start_x < end_x ? 1 : -1;
  const int dy = -std::abs(end_y - start_y);
  const int step_y = start_y < end_y ? 1 : -1;
  int error = dx + dy;
  while (x != end_x || y != end_y) {
    if (x >= 0 && y >= 0 && x < static_cast<int>(width) &&
      y < static_cast<int>(height))
    {
      free_cells[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] = true;
    }
    const int doubled_error = 2 * error;
    if (doubled_error >= dy) {
      error += dy;
      x += step_x;
    }
    if (doubled_error <= dx) {
      error += dx;
      y += step_y;
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 9) {
    std::cerr <<
      "Usage: pcd_to_occupancy INPUT.pcd OUTPUT_PREFIX [resolution] [min_height] "
      "[max_height] [pose_graph.g2o] [max_ray_range] [base_link_height]\n";
    return 2;
  }
  const std::filesystem::path input(argv[1]);
  const std::filesystem::path prefix(argv[2]);
  const double resolution = argc > 3 ? std::stod(argv[3]) : 0.05;
  const double min_height = argc > 4 ? std::stod(argv[4]) : 0.10;
  const double max_height = argc > 5 ? std::stod(argv[5]) : 1.80;
  const bool use_pose_graph = argc > 6;
  const double max_ray_range = argc > 7 ? std::stod(argv[7]) : 15.0;
  const double base_link_height = argc > 8 ? std::stod(argv[8]) : 0.171;
  if (resolution <= 0.0 || min_height >= max_height || max_ray_range <= 0.0 ||
    !std::isfinite(base_link_height))
  {
    std::cerr << "Invalid resolution, height slice, ray range, or base-link height\n";
    return 2;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(input.string(), cloud) != 0) {
    std::cerr << "Cannot load " << input << '\n';
    return 1;
  }

  std::vector<GraphPose> poses;
  try {
    if (use_pose_graph) {
      poses = load_graph_poses(argv[6]);
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }

  std::vector<ObstaclePoint> obstacles;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    std::size_t pose_index = 0;
    double height = point.z;
    if (use_pose_graph) {
      const auto [nearest_index, distance] = nearest_pose(point, poses);
      if (distance > max_ray_range) {
        continue;
      }
      pose_index = nearest_index;
      // The graph pose is base_link at body center. Convert point height to
      // the ground-projected base_footprint convention used by obstacle layers.
      height = point.z - poses[pose_index].z + base_link_height;
    }
    if (height < min_height || height > max_height) {
      continue;
    }
    obstacles.push_back(ObstaclePoint{point, pose_index});
    min_x = std::min(min_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_x = std::max(max_x, static_cast<double>(point.x));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }
  if (obstacles.empty()) {
    std::cerr << "No points in selected height slice\n";
    return 1;
  }
  for (const auto & pose : poses) {
    min_x = std::min(min_x, pose.x);
    min_y = std::min(min_y, pose.y);
    max_x = std::max(max_x, pose.x);
    max_y = std::max(max_y, pose.y);
  }

  constexpr double padding = 1.0;
  min_x -= padding;
  min_y -= padding;
  max_x += padding;
  max_y += padding;
  const auto width = static_cast<std::size_t>(std::ceil((max_x - min_x) / resolution)) + 1;
  const auto height = static_cast<std::size_t>(std::ceil((max_y - min_y) / resolution)) + 1;
  std::vector<bool> free_cells(width * height, false);
  std::vector<bool> occupied_cells(width * height, false);

  const auto grid_x = [min_x, resolution](double x) {
      return static_cast<int>(std::floor((x - min_x) / resolution));
    };
  const auto grid_y = [min_y, resolution](double y) {
      return static_cast<int>(std::floor((y - min_y) / resolution));
    };
  for (const auto & obstacle : obstacles) {
    const int obstacle_x = grid_x(obstacle.point.x);
    const int obstacle_y = grid_y(obstacle.point.y);
    if (obstacle_x < 0 || obstacle_y < 0 || obstacle_x >= static_cast<int>(width) ||
      obstacle_y >= static_cast<int>(height))
    {
      continue;
    }
    occupied_cells[static_cast<std::size_t>(obstacle_y) * width +
      static_cast<std::size_t>(obstacle_x)] = true;
    if (use_pose_graph) {
      const auto & pose = poses[obstacle.nearest_pose];
      mark_ray_free(
        grid_x(pose.x), grid_y(pose.y), obstacle_x, obstacle_y,
        width, height, free_cells);
    }
  }

  if (use_pose_graph) {
    const int free_radius_cells = static_cast<int>(std::ceil(0.35 / resolution));
    for (const auto & pose : poses) {
      const int center_x = grid_x(pose.x);
      const int center_y = grid_y(pose.y);
      for (int dy = -free_radius_cells; dy <= free_radius_cells; ++dy) {
        for (int dx = -free_radius_cells; dx <= free_radius_cells; ++dx) {
          if (dx * dx + dy * dy > free_radius_cells * free_radius_cells) {
            continue;
          }
          const int x = center_x + dx;
          const int y = center_y + dy;
          if (x >= 0 && y >= 0 && x < static_cast<int>(width) &&
            y < static_cast<int>(height))
          {
            free_cells[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] = true;
          }
        }
      }
    }
  }

  std::vector<std::uint8_t> pixels(width * height, 205U);
  std::size_t free_count = 0;
  std::size_t occupied_count = 0;
  for (std::size_t map_row = 0; map_row < height; ++map_row) {
    for (std::size_t column = 0; column < width; ++column) {
      const std::size_t map_index = map_row * width + column;
      const std::size_t image_index = (height - 1 - map_row) * width + column;
      if (occupied_cells[map_index]) {
        pixels[image_index] = 0U;
        ++occupied_count;
      } else if (free_cells[map_index]) {
        pixels[image_index] = 254U;
        ++free_count;
      }
    }
  }

  std::filesystem::create_directories(prefix.parent_path().empty() ? "." : prefix.parent_path());
  const auto pgm_path = prefix.string() + ".pgm";
  const auto yaml_path = prefix.string() + ".yaml";
  std::ofstream pgm(pgm_path, std::ios::binary);
  pgm << "P5\n" << width << ' ' << height << "\n255\n";
  pgm.write(reinterpret_cast<const char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  pgm.close();

  std::ofstream yaml(yaml_path);
  yaml << "image: " << std::filesystem::path(pgm_path).filename().string() << '\n';
  yaml << "mode: trinary\nresolution: " << resolution << '\n';
  yaml << "origin: [" << min_x << ", " << min_y << ", 0.0]\n";
  yaml << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
  yaml.close();

  const std::size_t unknown_count = width * height - free_count - occupied_count;
  std::cout << "Wrote " << pgm_path << " and " << yaml_path << " from " << obstacles.size()
            << " obstacle points. free=" << free_count << " occupied=" << occupied_count
            << " unknown=" << unknown_count << " mode="
            << (use_pose_graph ? "pose_graph_raycast" : "occupied_only") << '\n';
  return 0;
}
