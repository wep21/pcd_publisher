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

#ifndef PCD_PUBLISHER__PCD_PUBLISHER_HPP_
#define PCD_PUBLISHER__PCD_PUBLISHER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/io/pcd_io.h"

class PCDPublisher : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2Ptr = PointCloud2::SharedPtr;
  using PointCloud2ConstPtr = PointCloud2::ConstSharedPtr;
  PCDPublisher(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<PointCloud2>::SharedPtr point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Reader
  pcl::PCDReader reader_;

  // Parameter
  double rate_;
  std::string tf_frame_;
  std::string file_name_;
  PointCloud2Ptr output_;
};

#endif  // PCD_PUBLISHER__PCD_PUBLISHER_HPP_
