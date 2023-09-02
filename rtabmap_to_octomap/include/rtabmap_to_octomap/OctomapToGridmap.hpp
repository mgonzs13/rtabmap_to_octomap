
#ifndef RTABMAP_TO_OCTOMAP__OCTOMAPTOGRIDMAP_HPP_
#define RTABMAP_TO_OCTOMAP__OCTOMAPTOGRIDMAP_HPP_

#include <grid_map_ros/grid_map_ros.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace rtabmap_to_octomap {

/*!
 * Receives a volumetric OctoMap and converts it to a grid map with an elevation
 * layer. The grid map is published and can be viewed in Rviz.
 */
class OctomapToGridmap : public rclcpp::Node {
public:
  using GetOctomapSrv = octomap_msgs::srv::GetOctomap;
  using OctomapMessage = octomap_msgs::msg::Octomap;

  /*!
   * Constructor.
   */
  OctomapToGridmap();

  /*!
   * Destructor.
   */
  virtual ~OctomapToGridmap();

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  void convertAndPublishMap();

private:
  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPublisher_;

  //! Octomap publisher.
  rclcpp::Publisher<OctomapMessage>::SharedPtr octomapPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! Name of the grid map topic.
  std::string octomapServiceTopic_;

  //! Octomap service client
  rclcpp::Client<GetOctomapSrv>::SharedPtr client_;

  //! Bounding box of octomap to convert.
  float minX_;
  float maxX_;
  float minY_;
  float maxY_;
  float minZ_;
  float maxZ_;
};

} // namespace rtabmap_to_octomap
#endif // rtabmap_to_octomap__OCTOMAPTOGRIDMAP_HPP_