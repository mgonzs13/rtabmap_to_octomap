
#ifndef RTABMAP_TO_OCTOMAP__OCTOMAPTOGRIDMAP_HPP_
#define RTABMAP_TO_OCTOMAP__OCTOMAPTOGRIDMAP_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace rtabmap_to_octomap {

class OctomapToGridmap : public rclcpp::Node {
public:
  using OctomapMessage = octomap_msgs::msg::Octomap;

  OctomapToGridmap();
  virtual ~OctomapToGridmap();
  bool read_parameters();
  void octomap_cb(const OctomapMessage::SharedPtr msg);
  void convert_and_publish(const OctomapMessage::SharedPtr octomap);
  void create_occupancy();

private:
  grid_map::GridMap map_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
  rclcpp::Publisher<OctomapMessage>::SharedPtr octomap_publisher_;
  rclcpp::Subscription<OctomapMessage>::SharedPtr octo_sub_;

  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;
};

} // namespace rtabmap_to_octomap
#endif // rtabmap_to_octomap__OCTOMAPTOGRIDMAP_HPP_