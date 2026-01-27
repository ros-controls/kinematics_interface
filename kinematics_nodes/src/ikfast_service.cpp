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
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include "kinematics_interface/kinematics_interface.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "urdf/model.h"

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
  std::string group_name_;
  std::string base_link_;
  std::string tip_link_;
  size_t num_joints_;

  // Joint names from URDF (extracted from kinematic chain)
  std::vector<std::string> joint_names_;

  // Internal methods
  bool load_kinematics_plugin();
  bool validate_urdf_and_links();
  bool extract_joint_names_from_chain();
  bool validate_ik_request(const moveit_msgs::srv::GetPositionIK::Request::SharedPtr & request);
  moveit_msgs::msg::RobotState create_robot_state_msg(const std::vector<double> & joint_positions);

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
  this->declare_parameter<std::string>("group_name", "");
  this->declare_parameter<std::string>("base_link", "");
  this->declare_parameter<std::string>("tip_link", "");
  this->declare_parameter<double>("alpha", 0.000005);

  this->get_parameter("plugin_name", plugin_name_);
  this->get_parameter("robot_description", robot_description_);
  this->get_parameter("group_name", group_name_);
  this->get_parameter("base_link", base_link_);
  this->get_parameter("tip_link", tip_link_);

  // Validate required parameters
  if (base_link_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'base_link' is required but not set!");
    throw std::runtime_error("Missing required parameter: base_link");
  }

  if (tip_link_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'tip_link' is required but not set!");
    throw std::runtime_error("Missing required parameter: tip_link");
  }

  if (plugin_name_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'plugin_name' is required but not set!");
    throw std::runtime_error("Missing required parameter: plugin_name");
  }

  if (robot_description_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description' is required but not set!");
    throw std::runtime_error("Missing required parameter: robot_description");
  }

  RCLCPP_INFO(this->get_logger(), "Initializing IKFast Kinematics Service Node");
  RCLCPP_INFO(this->get_logger(), "  Plugin name: %s", plugin_name_.c_str());
  if (!group_name_.empty())
  {
    RCLCPP_INFO(this->get_logger(), "  Group name: %s", group_name_.c_str());
  }
  else
  {
    RCLCPP_WARN(
      this->get_logger(), "  Group name: NOT SET (will accept any group_name in requests)");
  }
  RCLCPP_INFO(this->get_logger(), "  Base link: %s", base_link_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Tip link: %s", tip_link_.c_str());

  // Validate URDF and link names
  if (!validate_urdf_and_links())
  {
    RCLCPP_ERROR(this->get_logger(), "URDF validation failed!");
    throw std::runtime_error("Invalid URDF or link names");
  }

  // Extract joint names from kinematic chain
  if (!extract_joint_names_from_chain())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract joint names from kinematic chain!");
    throw std::runtime_error("Invalid kinematic chain");
  }

  // Load kinematics plugin
  if (!load_kinematics_plugin())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load kinematics plugin!");
    throw std::runtime_error("Failed to load kinematics plugin");
  }

  // Create service
  get_ik_service_ = this->create_service<moveit_msgs::srv::GetPositionIK>(
    "compute_ikfast", std::bind(
                        &IKFastKinematicsServiceNode::get_position_ik_callback, this,
                        std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "IK service 'compute_ikfast' ready!");
}

bool IKFastKinematicsServiceNode::load_kinematics_plugin()
{
  try
  {
    RCLCPP_INFO(this->get_logger(), "Loading kinematics plugin: %s", plugin_name_.c_str());

    // Create plugin loader
    plugin_loader_ =
      std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        "kinematics_interface", "kinematics_interface::KinematicsInterface");

    // Load plugin instance
    kinematics_solver_ = plugin_loader_->createSharedInstance(plugin_name_);

    if (!kinematics_solver_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create plugin instance");
      return false;
    }

    // Initialize plugin with robot description and parameters
    bool init_success =
      kinematics_solver_->initialize(robot_description_, this->get_node_parameters_interface(), "");

    if (!init_success)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize kinematics plugin");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Kinematics plugin loaded and initialized successfully");
    return true;
  }
  catch (const pluginlib::PluginlibException & ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Plugin loading exception: %s", ex.what());
    return false;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception during plugin loading: %s", ex.what());
    return false;
  }
}

bool IKFastKinematicsServiceNode::validate_urdf_and_links()
{
  // Parse URDF
  if (!urdf_model_.initString(robot_description_))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF from robot_description parameter");
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "URDF parsed successfully for robot: %s", urdf_model_.getName().c_str());

  // Validate base_link exists in URDF
  auto base_link_ptr = urdf_model_.getLink(base_link_);
  if (!base_link_ptr)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Base link '%s' not found in URDF! Available links:", base_link_.c_str());

    // List available links for debugging
    std::stringstream available_links;
    for (const auto & link_pair : urdf_model_.links_)
    {
      available_links << link_pair.first << ", ";
    }
    RCLCPP_ERROR(this->get_logger(), "  Available: %s", available_links.str().c_str());
    return false;
  }

  // Validate tip_link exists in URDF
  auto tip_link_ptr = urdf_model_.getLink(tip_link_);
  if (!tip_link_ptr)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Tip link '%s' not found in URDF! Available links:", tip_link_.c_str());

    // List available links for debugging
    std::stringstream available_links;
    for (const auto & link_pair : urdf_model_.links_)
    {
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

bool IKFastKinematicsServiceNode::extract_joint_names_from_chain()
{
  // Traverse the kinematic chain from tip to base and extract joint names
  joint_names_.clear();

  std::string current_link = tip_link_;

  while (current_link != base_link_)
  {
    auto link_ptr = urdf_model_.getLink(current_link);
    if (!link_ptr)
    {
      RCLCPP_ERROR(
        this->get_logger(), "Link '%s' not found in chain traversal", current_link.c_str());
      return false;
    }

    auto parent_joint = link_ptr->parent_joint;
    if (!parent_joint)
    {
      RCLCPP_ERROR(
        this->get_logger(), "No parent joint found for link '%s'. Cannot reach base_link '%s'",
        current_link.c_str(), base_link_.c_str());
      return false;
    }

    // Only add revolute and prismatic joints (not fixed joints)
    if (
      parent_joint->type == urdf::Joint::REVOLUTE || parent_joint->type == urdf::Joint::PRISMATIC ||
      parent_joint->type == urdf::Joint::CONTINUOUS)
    {
      joint_names_.insert(joint_names_.begin(), parent_joint->name);
    }

    // Move to parent link
    current_link = parent_joint->parent_link_name;

    // Safety check to prevent infinite loops
    if (joint_names_.size() > 100)
    {
      RCLCPP_ERROR(
        this->get_logger(), "Kinematic chain too long (>100 joints). Possible loop in URDF?");
      return false;
    }
  }

  // Update num_joints based on actual chain
  num_joints_ = joint_names_.size();

  if (num_joints_ == 0)
  {
    RCLCPP_ERROR(
      this->get_logger(), "No movable joints found in chain from '%s' to '%s'", base_link_.c_str(),
      tip_link_.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Extracted %zu joints from kinematic chain:", num_joints_);
  std::stringstream ss;
  ss << "  Joints: [";
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    ss << joint_names_[i];
    if (i < joint_names_.size() - 1) ss << ", ";
  }
  ss << "]";
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  return true;
}

bool IKFastKinematicsServiceNode::validate_ik_request(
  const moveit_msgs::srv::GetPositionIK::Request::SharedPtr & request)
{
  // Validate group_name if configured
  if (!group_name_.empty() && request->ik_request.group_name != group_name_)
  {
    RCLCPP_ERROR(
      this->get_logger(), "IK request group_name '%s' does not match configured group_name '%s'",
      request->ik_request.group_name.c_str(), group_name_.c_str());
    return false;
  }

  // Validate that ik_link_name matches the configured tip_link
  if (request->ik_request.ik_link_name != tip_link_)
  {
    RCLCPP_ERROR(
      this->get_logger(), "IK request ik_link_name '%s' does not match configured tip_link '%s'",
      request->ik_request.ik_link_name.c_str(), tip_link_.c_str());
    return false;
  }

  // Validate that the frame_id in pose_stamped matches base_link
  const auto & frame_id = request->ik_request.pose_stamped.header.frame_id;
  if (!frame_id.empty() && frame_id != base_link_)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Pose frame_id '%s' does not match configured base_link '%s'",
      frame_id.c_str(), base_link_.c_str());
    return false;
  }

  // Validate seed state if provided
  const auto & seed_positions = request->ik_request.robot_state.joint_state.position;
  const auto & seed_names = request->ik_request.robot_state.joint_state.name;

  if (!seed_positions.empty())
  {
    // Check size matches expected number of joints
    if (seed_positions.size() != num_joints_)
    {
      RCLCPP_ERROR(
        this->get_logger(), "Seed state has %zu joint positions but kinematic chain has %zu joints",
        seed_positions.size(), num_joints_);
      return false;
    }

    // If joint names are provided in seed state, validate they match the chain
    if (!seed_names.empty())
    {
      if (seed_names.size() != seed_positions.size())
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Seed state joint names size (%zu) doesn't match positions size (%zu)", seed_names.size(),
          seed_positions.size());
        return false;
      }

      // Validate joint names match our kinematic chain
      for (size_t i = 0; i < seed_names.size(); ++i)
      {
        if (seed_names[i] != joint_names_[i])
        {
          RCLCPP_ERROR(
            this->get_logger(),
            "Seed state joint name mismatch at index %zu: expected '%s', got '%s'", i,
            joint_names_[i].c_str(), seed_names[i].c_str());
          RCLCPP_ERROR(
            this->get_logger(), "Expected joint order: [%s]",
            [this]()
            {
              std::stringstream ss;
              for (size_t j = 0; j < joint_names_.size(); ++j)
              {
                ss << joint_names_[j];
                if (j < joint_names_.size() - 1) ss << ", ";
              }
              return ss.str();
            }()
              .c_str());
          return false;
        }
      }
    }
  }

  return true;
}

void IKFastKinematicsServiceNode::get_position_ik_callback(
  const moveit_msgs::srv::GetPositionIK::Request::SharedPtr request,
  moveit_msgs::srv::GetPositionIK::Response::SharedPtr response)
{
  RCLCPP_INFO(
    this->get_logger(), "Received IK request for group '%s', link '%s'",
    request->ik_request.group_name.c_str(), request->ik_request.ik_link_name.c_str());

  // Validate the IK request
  if (!validate_ik_request(request))
  {
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
    RCLCPP_ERROR(
      this->get_logger(), "IK request validation failed - invalid link names or frame_id");
    return;
  }

  try
  {
    // Extract target pose from request and convert to Eigen
    const auto & pose_msg = request->ik_request.pose_stamped.pose;
    Eigen::Isometry3d target_pose;
    tf2::fromMsg(pose_msg, target_pose);

    RCLCPP_DEBUG(
      this->get_logger(), "Target pose - Position: [%.3f, %.3f, %.3f]", pose_msg.position.x,
      pose_msg.position.y, pose_msg.position.z);

    // Extract seed joint state (if provided)
    std::vector<double> seed_state;
    if (!request->ik_request.robot_state.joint_state.position.empty())
    {
      seed_state = request->ik_request.robot_state.joint_state.position;
      RCLCPP_DEBUG(this->get_logger(), "Using seed state with %zu joints", seed_state.size());
    }
    else
    {
      // Use zero initial state as seed if not provided
      seed_state.resize(num_joints_, 0.0);
      RCLCPP_DEBUG(this->get_logger(), "No seed state provided, using zeros");
    }

    // Call IK solver to find solution closest to seed state
    std::vector<double> solution;
    bool ik_success = kinematics_solver_->convert_cartesian_pose_to_closest_joint_state(
      target_pose, seed_state, solution);

    if (ik_success && !solution.empty())
    {
      // IK solution found - fill response
      response->solution = create_robot_state_msg(solution);
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

      RCLCPP_INFO(this->get_logger(), "IK solution found with %zu joints", solution.size());

      // Log solution for debugging
      std::stringstream ss;
      ss << "Solution: [";
      for (size_t i = 0; i < solution.size(); ++i)
      {
        ss << solution[i];
        if (i < solution.size() - 1) ss << ", ";
      }
      ss << "]";
      RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
    }
    else
    {
      // No IK solution found
      response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      RCLCPP_WARN(this->get_logger(), "No IK solution found for the requested pose");
    }
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception during IK computation: %s", ex.what());
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  }
}

moveit_msgs::msg::RobotState IKFastKinematicsServiceNode::create_robot_state_msg(
  const std::vector<double> & joint_positions)
{
  moveit_msgs::msg::RobotState robot_state;

  // Use actual joint names from URDF kinematic chain
  robot_state.joint_state.name = joint_names_;

  // Copy joint positions
  robot_state.joint_state.position = joint_positions;

  // Sanity check
  if (robot_state.joint_state.name.size() != robot_state.joint_state.position.size())
  {
    RCLCPP_ERROR(
      this->get_logger(), "Mismatch between joint names (%zu) and positions (%zu)",
      robot_state.joint_state.name.size(), robot_state.joint_state.position.size());
  }

  return robot_state;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<IKFastKinematicsServiceNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ikfast_service"), "Fatal error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
