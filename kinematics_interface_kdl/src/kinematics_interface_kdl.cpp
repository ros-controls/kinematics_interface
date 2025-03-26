// Copyright (c) 2022, PickNik, Inc.
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
//
/// \author: Andy Zelenak, Paul Gesel

#include "kinematics_interface_kdl/kinematics_interface_kdl.hpp"

namespace kinematics_interface_kdl
{
rclcpp::Logger LOGGER = rclcpp::get_logger("kinematics_interface_kdl");

bool KinematicsInterfaceKDL::initialize(
  const std::string & robot_description,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & param_namespace)
{
  // track initialization plugin
  initialized = true;

  // get parameters
  std::string ns = !param_namespace.empty() ? param_namespace + "." : "";

  std::string robot_description_local;
  if (robot_description.empty())
  {
    // If the robot_description input argument is empty, try to get the
    // robot_description from the node's parameters.
    auto robot_param = rclcpp::Parameter();
    if (!parameters_interface->get_parameter("robot_description", robot_param))
    {
      RCLCPP_ERROR(LOGGER, "parameter robot_description not set in kinematics_interface_kdl");
      return false;
    }
    robot_description_local = robot_param.as_string();
  }
  else
  {
    robot_description_local = robot_description;
  }

  // get alpha damping term
  auto alpha_param = rclcpp::Parameter("alpha", 0.000005);
  if (parameters_interface->has_parameter(ns + "alpha"))
  {
    parameters_interface->get_parameter(ns + "alpha", alpha_param);
  }
  alpha = alpha_param.as_double();
  // get end-effector name
  auto end_effector_name_param = rclcpp::Parameter("tip");
  if (parameters_interface->has_parameter(ns + "tip"))
  {
    parameters_interface->get_parameter(ns + "tip", end_effector_name_param);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Failed to find end effector name parameter [tip].");
    return false;
  }
  std::string end_effector_name = end_effector_name_param.as_string();

  // create kinematic chain
  KDL::Tree robot_tree;
  kdl_parser::treeFromString(robot_description_local, robot_tree);
  // get root name
  auto base_param = rclcpp::Parameter();
  if (parameters_interface->has_parameter(ns + "base"))
  {
    parameters_interface->get_parameter(ns + "base", base_param);
    root_name_ = base_param.as_string();
  }
  else
  {
    root_name_ = robot_tree.getRootSegment()->first;
  }

  if (!robot_tree.getChain(root_name_, end_effector_name, chain_))
  {
    RCLCPP_ERROR(
      LOGGER, "failed to find chain from robot root %s to end effector %s", root_name_.c_str(),
      end_effector_name.c_str());
    return false;
  }
  // create map from link names to their index
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
  {
    link_name_map_[chain_.getSegment(static_cast<unsigned int>(i)).getName()] =
      static_cast<int>(i) + 1;
  }
  // allocate dynamics memory
  num_joints_ = chain_.getNrOfJoints();
  q_ = KDL::JntArray(static_cast<unsigned int>(num_joints_));
  I = Eigen::MatrixXd(num_joints_, num_joints_);
  I.setIdentity();
  // create KDL solvers
  fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
  jacobian_ = std::make_shared<KDL::Jacobian>(num_joints_);
  jacobian_inverse_ = std::make_shared<Eigen::Matrix<double, Eigen::Dynamic, 6>>(num_joints_, 6);

  return true;
}

bool KinematicsInterfaceKDL::convert_joint_deltas_to_cartesian_deltas(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos,
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & delta_theta, const std::string & link_name,
  Eigen::Matrix<double, 6, 1> & delta_x)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_joint_vector(delta_theta))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  delta_x = jacobian_->data * delta_theta;

  return true;
}

bool KinematicsInterfaceKDL::convert_cartesian_deltas_to_joint_deltas(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos,
  const Eigen::Matrix<double, 6, 1> & delta_x, const std::string & link_name,
  Eigen::Matrix<double, Eigen::Dynamic, 1> & delta_theta)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_joint_vector(delta_theta))
  {
    return false;
  }

  // calculate Jacobian inverse
  if (!calculate_jacobian_inverse(joint_pos, link_name, *jacobian_inverse_))
  {
    return false;
  }

  delta_theta = *jacobian_inverse_ * delta_x;

  return true;
}

bool KinematicsInterfaceKDL::calculate_jacobian(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_jacobian(jacobian))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  jacobian = jacobian_->data;

  return true;
}

bool KinematicsInterfaceKDL::calculate_jacobian_inverse(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_jacobian_inverse(jacobian_inverse))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // calculate Jacobian
  jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = jacobian_->data;

  // damped inverse
  *jacobian_inverse_ =
    (jacobian.transpose() * jacobian + alpha * I).inverse() * jacobian.transpose();

  jacobian_inverse = *jacobian_inverse_;

  return true;
}

bool KinematicsInterfaceKDL::calculate_link_transform(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Isometry3d & transform)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name))
  {
    return false;
  }

  // get joint array
  q_.data = joint_pos;

  // reset transform_vec
  transform.setIdentity();

  // special case: since the root is not in the robot tree, need to return identity transform
  if (link_name == root_name_)
  {
    return true;
  }

  // create forward kinematics solver
  fk_pos_solver_->JntToCart(q_, frames_(0), link_name_map_[link_name]);
  tf2::transformKDLToEigen(frames_(0), transform);
  return true;
}

bool KinematicsInterfaceKDL::calculate_frame_difference(
  Eigen::Matrix<double, 7, 1> & x_a, Eigen::Matrix<double, 7, 1> & x_b, double dt,
  Eigen::Matrix<double, 6, 1> & delta_x)
{
  // verify inputs
  if (!verify_initialized() || !verify_period(dt))
  {
    return false;
  }

  // get frames
  frames_(0) = KDL::Frame(
    KDL::Rotation::Quaternion(x_a(3), x_a(4), x_a(5), x_a(6)), KDL::Vector(x_a(0), x_a(1), x_a(2)));
  frames_(1) = KDL::Frame(
    KDL::Rotation::Quaternion(x_b(3), x_b(4), x_b(5), x_b(6)), KDL::Vector(x_b(0), x_b(1), x_b(2)));

  // compute the difference
  delta_x_ = KDL::diff(frames_(0), frames_(1), dt);
  for (size_t i = 0; i < 6; ++i)
  {
    delta_x(static_cast<Eigen::Index>(i)) = delta_x_[static_cast<int>(i)];
  }

  return true;
}

bool KinematicsInterfaceKDL::verify_link_name(const std::string & link_name)
{
  if (link_name == root_name_)
  {
    return true;
  }
  if (link_name_map_.find(link_name) == link_name_map_.end())
  {
    std::string links;
    for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
      links += "\n" + chain_.getSegment(static_cast<unsigned int>(i)).getName();
    }
    RCLCPP_ERROR(
      LOGGER, "The link %s was not found in the robot chain. Available links are: %s",
      link_name.c_str(), links.c_str());
    return false;
  }
  return true;
}

bool KinematicsInterfaceKDL::verify_joint_vector(const Eigen::VectorXd & joint_vector)
{
  if (static_cast<size_t>(joint_vector.size()) != num_joints_)
  {
    RCLCPP_ERROR(
      LOGGER, "Invalid joint vector size (%zu). Expected size is %zu.", joint_vector.size(),
      num_joints_);
    return false;
  }
  return true;
}

bool KinematicsInterfaceKDL::verify_initialized()
{
  // check if interface is initialized
  if (!initialized)
  {
    RCLCPP_ERROR(
      LOGGER,
      "The KDL kinematics plugin was not initialized. Ensure you called the initialize method.");
    return false;
  }
  return true;
}

bool KinematicsInterfaceKDL::verify_jacobian(
  const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  if (jacobian.rows() != jacobian_->rows() || jacobian.cols() != jacobian_->columns())
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the jacobian (%zu, %zu) does not match the required size of (%u, %u)",
      jacobian.rows(), jacobian.cols(), jacobian_->rows(), jacobian_->columns());
    return false;
  }
  return true;
}

bool KinematicsInterfaceKDL::verify_jacobian_inverse(
  const Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse)
{
  if (
    jacobian_inverse.rows() != jacobian_->columns() || jacobian_inverse.cols() != jacobian_->rows())
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the jacobian (%zu, %zu) does not match the required size of (%u, %u)",
      jacobian_inverse.rows(), jacobian_inverse.cols(), jacobian_->columns(), jacobian_->rows());
    return false;
  }
  return true;
}

bool KinematicsInterfaceKDL::verify_period(const double dt)
{
  if (dt < 0)
  {
    RCLCPP_ERROR(LOGGER, "The period (%f) must be a non-negative number", dt);
    return false;
  }
  return true;
}

}  // namespace kinematics_interface_kdl

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kinematics_interface_kdl::KinematicsInterfaceKDL, kinematics_interface::KinematicsInterface)
