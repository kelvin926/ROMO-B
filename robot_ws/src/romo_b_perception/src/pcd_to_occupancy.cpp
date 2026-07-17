#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 6) {
    std::cerr << "Usage: pcd_to_occupancy INPUT.pcd OUTPUT_PREFIX [resolution] [min_z] [max_z]\n";
    return 2;
  }
  const std::filesystem::path input(argv[1]);
  const std::filesystem::path prefix(argv[2]);
  const double resolution = argc > 3 ? std::stod(argv[3]) : 0.05;
  const double min_z = argc > 4 ? std::stod(argv[4]) : 0.10;
  const double max_z = argc > 5 ? std::stod(argv[5]) : 1.80;
  if (resolution <= 0.0 || min_z >= max_z) {
    std::cerr << "Invalid resolution or height slice\n";
    return 2;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(input.string(), cloud) != 0) {
    std::cerr << "Cannot load " << input << '\n';
    return 1;
  }

  std::vector<pcl::PointXYZ> points;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      point.z < min_z || point.z > max_z)
    {
      continue;
    }
    points.push_back(point);
    min_x = std::min(min_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_x = std::max(max_x, static_cast<double>(point.x));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }
  if (points.empty()) {
    std::cerr << "No points in selected height slice\n";
    return 1;
  }

  constexpr double padding = 1.0;
  min_x -= padding;
  min_y -= padding;
  max_x += padding;
  max_y += padding;
  const auto width = static_cast<std::size_t>(std::ceil((max_x - min_x) / resolution)) + 1;
  const auto height = static_cast<std::size_t>(std::ceil((max_y - min_y) / resolution)) + 1;
  std::vector<std::uint8_t> pixels(width * height, 254U);
  for (const auto & point : points) {
    const auto column = static_cast<std::size_t>((point.x - min_x) / resolution);
    const auto map_row = static_cast<std::size_t>((point.y - min_y) / resolution);
    if (column < width && map_row < height) {
      const auto image_row = height - 1 - map_row;
      pixels[image_row * width + column] = 0U;
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

  std::cout << "Wrote " << pgm_path << " and " << yaml_path << " from " << points.size()
            << " obstacle points. Inspect free-space assumptions before navigation.\n";
  return 0;
}
