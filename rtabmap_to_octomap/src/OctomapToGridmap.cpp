#include "rtabmap_to_octomap/OctomapToGridmap.hpp"

#include <grid_map_octomap/GridMapOctomapConverter.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

using std::placeholders::_1;

using namespace rtabmap_to_octomap;

OctomapToGridmap::OctomapToGridmap()
    : Node("octomap_to_gridmap_demo"),
      map_(grid_map::GridMap({"elevation", "occupancy"})) {

  read_parameters();

  this->map_.setBasicLayers({"elevation", "occupancy"});

  this->grid_map_publisher_ =
      this->create_publisher<grid_map_msgs::msg::GridMap>(
          "grid_map", rclcpp::QoS(1).transient_local());
  this->octomap_publisher_ = this->create_publisher<OctomapMessage>(
      "octomap", rclcpp::QoS(1).transient_local());

  this->octo_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "octomap_full", 1, std::bind(&OctomapToGridmap::octomap_cb, this, _1));
}

OctomapToGridmap::~OctomapToGridmap() {}

void OctomapToGridmap::octomap_cb(const OctomapMessage::SharedPtr msg) {
  this->convert_and_publish(msg);
}

bool OctomapToGridmap::read_parameters() {
  this->declare_parameter("min_x", NAN);
  this->declare_parameter("max_x", NAN);
  this->declare_parameter("min_y", NAN);
  this->declare_parameter("max_y", NAN);
  this->declare_parameter("min_z", NAN);
  this->declare_parameter("max_z", NAN);

  this->get_parameter("min_x", this->minX_);
  this->get_parameter("max_x", this->maxX_);
  this->get_parameter("min_y", this->minY_);
  this->get_parameter("max_y", this->maxY_);
  this->get_parameter("min_z", this->minZ_);
  this->get_parameter("max_z", this->maxZ_);

  return true;
}

void OctomapToGridmap::convert_and_publish(
    const OctomapMessage::SharedPtr msg) {

  octomap::OcTree *octomap = nullptr;
  octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
  if (tree) {
    octomap = dynamic_cast<octomap::OcTree *>(tree);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call convert Octomap.");
    return;
  }

  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;
  octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));
  if (!std::isnan(this->minX_)) {
    min_bound(0) = this->minX_;
  }
  if (!std::isnan(maxX_)) {
    max_bound(0) = this->maxX_;
  }
  if (!std::isnan(minY_)) {
    min_bound(1) = this->minY_;
  }
  if (!std::isnan(maxY_)) {
    max_bound(1) = this->maxY_;
  }
  if (!std::isnan(minZ_)) {
    min_bound(2) = this->minZ_;
  }
  if (!std::isnan(maxZ_)) {
    max_bound(2) = this->maxZ_;
  }

  bool res = grid_map::GridMapOctomapConverter::fromOctomap(
      *octomap, "elevation", this->map_, &min_bound, &max_bound);
  if (!res) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to call convert Octomap elevation layer.");
    return;
  }

  this->map_.setFrameId(msg->header.frame_id);

  for (auto i = 0; i < this->map_.getSize()(0); i++) {
    for (auto j = 0; j < this->map_.getSize()(1); j++) {
      grid_map::Index idx(i, j);
      float height = this->map_.at("elevation", idx);

      if (std::isnan(height)) {
        this->map_.at("elevation", idx) = min_bound(2);
      }
    }
  }

  this->create_occupancy();

  // Publish as grid map.
  auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(this->map_);

  grid_map_msg->basic_layers = {};

  if (grid_map_msg->data[0].data.size() == 0) {
    return;
  }

  this->grid_map_publisher_->publish(std::move(grid_map_msg));

  // Also publish as an octomap msg for visualization
  OctomapMessage octomap_msg;
  octomap_msgs::fullMapToMsg(*octomap, octomap_msg);
  octomap_msg.header.frame_id = map_.getFrameId();

  std::unique_ptr<OctomapMessage> octomap_msg_ptr(
      new OctomapMessage(octomap_msg));
  this->octomap_publisher_->publish(std::move(octomap_msg_ptr));
}

void OctomapToGridmap::create_occupancy() {
  float res = static_cast<float>(this->map_.getResolution());

  for (auto i = 0; i < this->map_.getSize()(0); i++) {
    for (auto j = 0; j < this->map_.getSize()(1); j++) {
      grid_map::Index idx(i, j);
      this->map_.at("occupancy", idx) = 1.0;
    }
  }

  for (auto i = 0; i < this->map_.getSize()(0); i++) {
    for (auto j = 0; j < this->map_.getSize()(1); j++) {
      grid_map::Index idx(i, j);
      float height = this->map_.at("elevation", idx);

      float diff_n(0.0), diff_s(0.0), diff_w(0.0), diff_e(0.0);

      // Check N cell
      grid_map::Index n_pos(idx);
      n_pos(0) = n_pos(0) + 1;
      if (this->map_.isValid(n_pos, "elevation")) {
        diff_n = abs(this->map_.at("elevation", n_pos) - height);
      }

      grid_map::Index w_pos(idx);
      w_pos(1) = w_pos(1) + 1;
      if (this->map_.isValid(w_pos, "elevation")) {
        diff_w = abs(this->map_.at("elevation", w_pos) - height);
      }

      grid_map::Index e_pos(idx);
      e_pos(1) = e_pos(1) - 1;
      if (this->map_.isValid(e_pos, "elevation")) {
        diff_e = abs(this->map_.at("elevation", e_pos) - height);
      }

      grid_map::Index s_pos(idx);
      s_pos(0) = s_pos(0) - 1;
      if (this->map_.isValid(s_pos, "elevation")) {
        diff_s = abs(this->map_.at("elevation", s_pos) - height);
      }

      if (diff_n > (res * 2.0) || diff_s > (res * 2.0) ||
          diff_w > (res * 2.0) || diff_e > (res * 2.0)) {
        this->map_.at("occupancy", idx) = 254.0;
      }
    }
  }
}
