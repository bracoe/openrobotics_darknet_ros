// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef OPENROBOTICS_DARKNET_ROS__DETECTOR_NODE_HPP_
#define OPENROBOTICS_DARKNET_ROS__DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <DarkHelp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace openrobotics
{
namespace darknet_ros
{

class DetectorNode : public rclcpp::Node
{
public:
  /// \brief Create a node that uses ROS parameters to get the network
  explicit DetectorNode(rclcpp::NodeOptions options);

  virtual ~DetectorNode();

private:
  void on_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  
  std::unique_ptr<DarkHelp::NN> network_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  std::string sub_topic_;
  
  float threshold_{0.5f};
  float nms_threshold_{0.45f};
  rcl_interfaces::msg::ParameterDescriptor threshold_desc_;
  rcl_interfaces::msg::ParameterDescriptor nms_threshold_desc_;
};
}  // namespace darknet_ros
}  // namespace openrobotics

#endif  // OPENROBOTICS_DARKNET_ROS__DETECTOR_NODE_HPP_
