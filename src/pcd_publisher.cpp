// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pcd_publisher/pcd_publisher.hpp"

#include "rclcpp/create_timer.hpp"

#include "pcl_conversions/pcl_conversions.h"

PCDPublisher::PCDPublisher(const rclcpp::NodeOptions & node_options)
: Node("pcd_publisher", node_options),
  rate_(declare_parameter("rate", 10.0)),
  tf_frame_(declare_parameter("tf_frame", "base_link")),
  file_name_(declare_parameter("file_name").template get<std::string>()),
  output_(std::make_shared<PointCloud2>())
{
  point_pub_ = this->create_publisher<PointCloud2>("output", rclcpp::SensorDataQoS());
  pcl::PCLPointCloud2 cloud;
  if (reader_.read(file_name_, cloud) < 0)
  {
    RCLCPP_ERROR(get_logger(), "Error reading %s !", file_name_.c_str());
    return;
  }
  pcl_conversions::moveFromPCL(cloud, *(output_));
  output_->header.frame_id = tf_frame_;
  // Timer
  auto on_timer =
    [this]() -> void {
      output_->header.stamp = this->now();
      point_pub_->publish(*output_);
    };
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
    this->get_clock(), period, std::move(on_timer),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PCDPublisher)
