#include "rtabmap_to_octomap/OctomapToGridmap.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto OctomapToGridmap =
      std::make_shared<rtabmap_to_octomap::OctomapToGridmap>();

  rclcpp::sleep_for(std::chrono::seconds(2));

  rclcpp::Rate r(0.1); // 1 hz
  while (rclcpp::ok()) {
    OctomapToGridmap->convertAndPublishMap();
    rclcpp::spin_some(OctomapToGridmap);
    r.sleep();
  }
  return 0;
}