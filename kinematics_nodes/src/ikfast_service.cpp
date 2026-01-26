// Copyright (c) 2026 b»robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential
//
// Adapted for <Insert_Company_Name> that received unlimited, worldwide
// use and change right, except distributing this library separately
// of their product.

#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "urdf/model.h"
#include <Eigen/Geometry>

class IKFastKinematicsServiceNode : public rclcpp::Node
{
private:
  // Plugin management
  std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> plugin_loader_;
  std::shared_ptr<kinematics_interface::KinematicsInterface> kinematics_solver_;

  // ROS2 service - moveit compatible
  rclcpp::Service<moveit_msgs::srv::GetPositionIK>::SharedPtr get_ik_service_;

  // URDF model for validation
  urdf::Model urdf_model_;

  // Parameters
  std::string plugin_name_;
  std::string robot_description_;
  std::string base_link_;
  std::string tip_link_;
  size_t num_joints_;

  // Internal methods
  bool load_kinematics_plugin();
  bool validate_urdf_and_links();
  bool validate_ik_request(const moveit_msgs::srv::GetPositionIK::Request::SharedPtr & request);
  moveit_msgs::msg::RobotState create_robot_state_msg(
    const std::vector<double> & joint_positions);

public:
  explicit IKFastKinematicsServiceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Service callbacks
  void get_position_ik_callback(
    const moveit_msgs::srv::GetPositionIK::Request::SharedPtr request,
    moveit_msgs::srv::GetPositionIK::Response::SharedPtr response);
};

IKFastKinematicsServiceNode::IKFastKinematicsServiceNode(const rclcpp::NodeOptions & options)
: Node("ikfast_service_node", options), num_joints_(6)
{
  // Declare and read parameters
  this->declare_parameter<std::string>("plugin_name", "");
  this->declare_parameter<std::string>("robot_description", "");
  this->declare_parameter<std::string>("base_link", "base_link");
  this->declare_parameter<std::string>("tip_link", "link_6");
  this->declare_parameter<double>("alpha", 0.000005);

  this->get_parameter("plugin_name", plugin_name_);
  this->get_parameter("robot_description", robot_description_);
  this->get_parameter("base_link", base_link_);
  this->get_parameter("tip_link", tip_link_);

  // Validate required parameters
  if (plugin_name_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'plugin_name' is required but not set!");
    throw std::runtime_error("Missing required parameter: plugin_name");
  }

  if (robot_description_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description' is required but not set!");
    throw std::runtime_error("Missing required parameter: robot_description");
  }

  RCLCPP_INFO(this->get_logger(), "Initializing IKFast Kinematics Service Node");
  RCLCPP_INFO(this->get_logger(), "  Plugin name: %s", plugin_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Base link: %s", base_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Tip link: %s", tip_link_.c_str());

  // Validate URDF and link names
  if (!validate_urdf_and_links()) {
    RCLCPP_ERROR(this->get_logger(), "URDF validation failed!");
    throw std::runtime_error("Invalid URDF or link names");
  }

  // Load kinematics plugin
  if (!load_kinematics_plugin()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load kinematics plugin!");
    throw std::runtime_error("Failed to load kinematics plugin");
  }

  // Create service
  get_ik_service_ = this->create_service<moveit_msgs::srv::GetPositionIK>(
    "compute_ikfast",
    std::bind(&IKFastKinematicsServiceNode::get_position_ik_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "IK service 'compute_ikfast' ready!");
}

bool IKFastKinematicsServiceNode::load_kinematics_plugin()
{
  try {
    RCLCPP_INFO(this->get_logger(), "Loading kinematics plugin: %s", plugin_name_.c_str());

    // Create plugin loader
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");

    // Load plugin instance
    kinematics_solver_ = plugin_loader_->createSharedInstance(plugin_name_);

    if (!kinematics_solver_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create plugin instance");
      return false;
    }

    // Initialize plugin with robot description and parameters
    bool init_success = kinematics_solver_->initialize(
      robot_description_,
      this->get_node_parameters_interface(),
      "");

    if (!init_success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize kinematics plugin");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Kinematics plugin loaded and initialized successfully");
    return true;

  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Plugin loading exception: %s", ex.what());
    return false;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "Exception during plugin loading: %s", ex.what());
    return false;
  }
}

bool IKFastKinematicsServiceNode::validate_urdf_and_links()
{
  // Parse URDF
  if (!urdf_model_.initString(robot_description_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF from robot_description parameter");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "URDF parsed successfully for robot: %s", urdf_model_.getName().c_str());

  // Validate base_link exists in URDF
  auto base_link_ptr = urdf_model_.getLink(base_link_);
  if (!base_link_ptr) {
    RCLCPP_ERROR(this->get_logger(), 
                 "Base link '%s' not found in URDF! Available links:", 
                 base_link_.c_str());
    
    // List available links for debugging
    std::stringstream available_links;
    for (const auto & link_pair : urdf_model_.links_) {
      available_links << link_pair.first << ", ";
    }
    RCLCPP_ERROR(this->get_logger(), "  Available: %s", available_links.str().c_str());
    return false;
  }

  // Validate tip_link exists in URDF
  auto tip_link_ptr = urdf_model_.getLink(tip_link_);
  if (!tip_link_ptr) {
    RCLCPP_ERROR(this->get_logger(), 
                 "Tip link '%s' not found in URDF! Available links:", 
                 tip_link_.c_str());
    
    // List available links for debugging
    std::stringstream available_links;
    for (const auto & link_pair : urdf_model_.links_) {
      available_links << link_pair.first << ", ";
    }
    RCLCPP_ERROR(this->get_logger(), "  Available: %s", available_links.str().c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Link validation successful:");
  RCLCPP_INFO(this->get_logger(), "  Base link '%s' found in URDF", base_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Tip link '%s' found in URDF", tip_link_.c_str());
  
  return true;
}

bool IKFastKinematicsServiceNode::validate_ik_request(
  const moveit_msgs::srv::GetPositionIK::Request::SharedPtr & request)
{
  // Validate that ik_link_name matches the configured tip_link
  if (request->ik_request.ik_link_name != tip_link_) {
    RCLCPP_WARN(this->get_logger(),
                "IK request link name '%s' does not match configured tip link '%s'",
                request->ik_request.ik_link_name.c_str(),
                tip_link_.c_str());
    return false;
  }

  // Validate that the frame_id in pose_stamped matches base_link
  const auto & frame_id = request->ik_request.pose_stamped.header.frame_id;
  if (!frame_id.empty() && frame_id != base_link_) {
    RCLCPP_WARN(this->get_logger(),
                "Pose frame_id '%s' does not match configured base link '%s'",
                frame_id.c_str(),
                base_link_.c_str());
    return false;
  }

  return true;
}

void IKFastKinematicsServiceNode::get_position_ik_callback(
  const moveit_msgs::srv::GetPositionIK::Request::SharedPtr request,
  moveit_msgs::srv::GetPositionIK::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "Received IK request for group '%s', link '%s'",
              request->ik_request.group_name.c_str(),
              request->ik_request.ik_link_name.c_str());

  // Validate the IK request
  if (!validate_ik_request(request)) {
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
    RCLCPP_ERROR(this->get_logger(), "IK request validation failed - invalid link names or frame_id");
    return;
  }

  try {
    // Extract target pose from request and convert to Eigen
    const auto & pose_msg = request->ik_request.pose_stamped.pose;
    Eigen::Isometry3d target_pose;
    tf2::fromMsg(pose_msg, target_pose);

    RCLCPP_DEBUG(this->get_logger(), "Target pose - Position: [%.3f, %.3f, %.3f]",
                pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

    // Extract seed joint state (if provided)
    std::vector<double> seed_state;
    if (!request->ik_request.robot_state.joint_state.position.empty()) {
      seed_state = request->ik_request.robot_state.joint_state.position;
      RCLCPP_DEBUG(this->get_logger(), "Using seed state with %zu joints", seed_state.size());
    } else {
      // Use zero initial state as seed if not provided
      seed_state.resize(num_joints_, 0.0);
      RCLCPP_DEBUG(this->get_logger(), "No seed state provided, using zeros");
    }

    // Call IK solver to find solution closest to seed state
    std::vector<double> solution;
    bool ik_success = kinematics_solver_->convert_cartesian_pose_to_closest_joint_state(
      target_pose, seed_state, solution);

    if (ik_success && !solution.empty()) {
      // IK solution found - fill response
      response->solution = create_robot_state_msg(solution);
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

      RCLCPP_INFO(this->get_logger(), "IK solution found with %zu joints", solution.size());
      
      // Log solution for debugging
      std::stringstream ss;
      ss << "Solution: [";
      for (size_t i = 0; i < solution.size(); ++i) {
        ss << solution[i];
        if (i < solution.size() - 1) ss << ", ";
      }
      ss << "]";
      RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());

    } else {
      // No IK solution found
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      RCLCPP_WARN(this->get_logger(), "No IK solution found for the requested pose");
    }

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "Exception during IK computation: %s", ex.what());
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
}

moveit_msgs::msg::RobotState IKFastKinematicsServiceNode::create_robot_state_msg(
  const std::vector<double> & joint_positions)
{
  moveit_msgs::msg::RobotState robot_state;

  // Create generic joint names (joint_1, joint_2, ...)
  robot_state.joint_state.name.resize(joint_positions.size());
  for (size_t i = 0; i < joint_positions.size(); ++i) {
    robot_state.joint_state.name[i] = "joint_" + std::to_string(i + 1);
  }

  // Copy joint positions
  robot_state.joint_state.position = joint_positions;

  return robot_state;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<IKFastKinematicsServiceNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("ikfast_service"), "Fatal error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
