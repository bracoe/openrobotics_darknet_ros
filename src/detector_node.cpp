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

#include "openrobotics_darknet_ros/detector_node.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/parameter_value.hpp"
#include <memory>
#include <string>
#include <vector>

namespace openrobotics
{
namespace darknet_ros
{

DetectorNode::DetectorNode(rclcpp::NodeOptions options)
: rclcpp::Node("detector_node", options)
{
  // Read-only input parameters: cfg, weights, classes
  rcl_interfaces::msg::ParameterDescriptor network_cfg_desc;
  network_cfg_desc.description = "Path to config file describing network";
  network_cfg_desc.read_only = true;
  const std::string network_config_path = declare_parameter<std::string>(
    "network.config", "", network_cfg_desc);
  RCLCPP_INFO(this->get_logger(), "Network config path: %s", network_config_path.c_str());

  rcl_interfaces::msg::ParameterDescriptor network_weights_desc;
  network_weights_desc.description = "Path to file describing network weights";
  network_weights_desc.read_only = true;
  const std::string network_weights_path = declare_parameter<std::string>(
    "network.weights", "", network_weights_desc);
  RCLCPP_INFO(this->get_logger(), "Network weights path: %s", network_weights_path.c_str());

  rcl_interfaces::msg::ParameterDescriptor network_class_names_desc;
  network_class_names_desc.description = "Path to file with class names (one per line)";
  network_class_names_desc.read_only = true;
  const std::string network_class_names_path = declare_parameter<std::string>(
    "network.class_names", "", network_class_names_desc);
  RCLCPP_INFO(this->get_logger(), "Network class names path: %s", network_class_names_path.c_str());

  RCLCPP_INFO(this->get_logger(), "Initializing DarkHelp neural network...");
  network_ = std::make_unique<DarkHelp::NN>(
    network_config_path,
    network_weights_path,
    network_class_names_path);
  RCLCPP_INFO(this->get_logger(), "DarkHelp network initialized successfully");

  threshold_desc_.description = "Minimum detection confidence [0.0, 1.0]";
  threshold_desc_.name = "detection.threshold";
  threshold_ = declare_parameter(
    threshold_desc_.name,
    threshold_,
    threshold_desc_);

  nms_threshold_desc_.description =
    "Non Maximal Suppression threshold for filtering overlapping boxes [0.0, 1.0]";
  nms_threshold_desc_.name = "detection.nms_threshold";
  nms_threshold_ = declare_parameter(
    nms_threshold_desc_.name,
    nms_threshold_,
    nms_threshold_desc_);
  // Set DarkHelp thresholds
  network_->config.threshold = threshold_;
  network_->config.non_maximal_suppression_threshold = nms_threshold_;

  RCLCPP_INFO(this->get_logger(), "Detection thresholds:");
  RCLCPP_INFO(this->get_logger(), "  Confidence threshold: %.2f", threshold_);
  RCLCPP_INFO(this->get_logger(), "  NMS threshold: %.2f", nms_threshold_);


  // Ouput topic ~/detections [vision_msgs/msg/Detection2DArray]
  detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    "~/detections", 1);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "~/images",
    rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::on_image_callback, this, std::placeholders::_1));
}

DetectorNode::~DetectorNode()
{
}

void DetectorNode::on_image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try {
    // Convert and predict
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    const auto & results = network_->predict(cv_ptr->image);
    
    // Convert detections to ROS message
    vision_msgs::msg::Detection2DArray detection_msg;
    detection_msg.header = msg->header;
    detection_msg.detections.reserve(results.size());
    
    for (const auto & prediction : results) {
      vision_msgs::msg::Detection2D detection;
      
      // bounding box
      const float center_x = prediction.rect.x + prediction.rect.width * 0.5f;
      const float center_y = prediction.rect.y + prediction.rect.height * 0.5f;
      detection.bbox.center.position.x = center_x;
      detection.bbox.center.position.y = center_y;
      detection.bbox.size_x = prediction.rect.width;
      detection.bbox.size_y = prediction.rect.height;
      
      // result
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = std::to_string(prediction.best_class);
      hyp.hypothesis.score = prediction.best_probability;
      detection.results.emplace_back(std::move(hyp));
      
      detection_msg.detections.emplace_back(std::move(detection));
    }
    
    detections_pub_->publish(std::move(detection_msg));
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

}  // namespace darknet_ros
}  // namespace openrobotics

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(openrobotics::darknet_ros::DetectorNode)
