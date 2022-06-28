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

#include "kdl_plugin/kdl_plugin.hpp"

namespace kdl_plugin {

  rclcpp::Logger LOGGER = rclcpp::get_logger("kdl_plugin");

  bool KDLKinematics::initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                                 const std::string &end_effector_name) {
    initialized = true;
    end_effector_name_ = end_effector_name;
    // get robot description
    auto robot_param = rclcpp::Parameter();
    if (!node->get_parameter("robot_description", robot_param)) {
      RCLCPP_ERROR(LOGGER, "parameter robot_description not set");
      return false;
    }
    auto robot_description = robot_param.as_string();
    // get alpha damping term
    auto alpha_param = rclcpp::Parameter("alpha", 0.000005);
    if (node->has_parameter("alpha")) {
      node->get_parameter("alpha", alpha_param);
    }
    alpha = alpha_param.as_double();
    // create kinematic chain
    KDL::Tree robot_tree;
    kdl_parser::treeFromString(robot_description, robot_tree);
    root_name_ = robot_tree.getRootSegment()->first;
    if (!robot_tree.getChain(root_name_, end_effector_name, chain_)) {
      RCLCPP_ERROR(LOGGER, "failed to find chain from robot root %s to end effector %s",
                   root_name_.c_str(), end_effector_name.c_str());
      return false;
    }
    //create map from link names to their index
    for (auto i = 0u; i < chain_.getNrOfSegments(); i++) {
      link_name_map_[chain_.getSegment(i).getName()] = i + 1;
    }
    // allocate dynamics memory
    num_joints_ = chain_.getNrOfJoints();
    q_ = KDL::JntArray(num_joints_);
    delta_theta = Eigen::VectorXd(num_joints_);
    I = Eigen::MatrixXd(num_joints_, num_joints_);
    I.setIdentity();
    // create KDL solvers
    fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
    jacobian_ = std::make_shared<KDL::Jacobian>(num_joints_);

    return true;
  }


  bool KDLKinematics::convert_joint_deltas_to_cartesian_deltas(const std::vector<double> &joint_pos,
                                                               const std::vector<double> &delta_theta_vec,
                                                               const std::string &link_name,
                                                               std::vector<double> &delta_x_vec) {
    // get joint array and check dimensions
    if (!update_joint_array(joint_pos)) {
      return false;
    }
    if (!verify_link_name(link_name)){
      return false;
    }
    // copy vector to eigen type
    memcpy(delta_theta.data(), delta_theta_vec.data(), delta_theta_vec.size()*sizeof(double));
    // calculate Jacobian
    jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
    delta_x = jacobian_->data * delta_theta;
    // copy eigen type to vector
    memcpy(delta_x_vec.data(), delta_x.data(), 6*sizeof(double));
    return true;
  }

  bool KDLKinematics::convert_cartesian_deltas_to_joint_deltas(const std::vector<double> &joint_pos,
                                                               const std::vector<double> &delta_x_vec,
                                                               const std::string &link_name,
                                                               std::vector<double> &delta_theta_vec) {
    // get joint array and check dimensions
    if (!update_joint_array(joint_pos)) {
      return false;
    }
    if (!verify_link_name(link_name)){
      return false;
    }
    // copy vector to eigen type
    memcpy(delta_x.data(), delta_x_vec.data(), delta_x_vec.size()*sizeof(double));
    // calculate Jacobian
    jac_solver_->JntToJac(q_, *jacobian_, link_name_map_[link_name]);
    // TODO this dynamic allocation needs to be replaced
    Eigen::Matrix<double, 6, Eigen::Dynamic> J = jacobian_->data;
    // damped inverse
    Eigen::Matrix<double, 6, 6> J_inverse = (J.transpose() * J + alpha * I).inverse() * J.transpose();
    delta_theta = J_inverse * delta_x;
    // copy eigen type to vector
    memcpy(delta_theta_vec.data(), delta_theta.data(), num_joints_*sizeof(double));
    return true;
  }

  bool KDLKinematics::calculate_link_transform(const std::vector<double> &joint_pos,
                                               const std::string &link_name,
                                               std::vector<double> &transform_vec
  ) {
    // get joint array and check dimensions
    if (!update_joint_array(joint_pos)) {
      return false;
    }
    if (!verify_link_name(link_name)){
      return false;
    }

    // reset transform_vec
    memset(transform_vec.data(), 0, 16*sizeof(double));

    // special case: since the root is not in the robot tree, need to return identity transform
    if (link_name == root_name_) {
      transform_vec[0] = 1.0;
      transform_vec[5] = 1.0;
      transform_vec[10] = 1.0;
      transform_vec[15] = 1.0;
      return true;
    }
    fk_pos_solver_->JntToCart(q_, frame_, link_name_map_[link_name]);
    double tmp[] = {frame_.p.x(), frame_.p.y(), frame_.p.z()};
    // KDL::Rotation stores data in row-major format. e.g Xx, Yz, Zx, Xy... = data index at 0, 1, 2, 3, 4...
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        transform_vec[r + 4 * c] = frame_.M.data[3 * r + c];
      }
      transform_vec[4 * 3 + r] = tmp[r];
    }
    transform_vec[15] = 1.0;
    return true;
  }

  bool KDLKinematics::update_joint_array(const std::vector<double> &joint_pos) {
    // check if interface is initialized
    if (!initialized) {
      RCLCPP_ERROR(LOGGER, "The KDL kinematics plugin was not initialized. Ensure you called the initialize method.");
      return false;
    }
    // check that joint_pos_ is the correct size
    if (joint_pos.size() != num_joints_) {
      RCLCPP_ERROR(LOGGER, "The size of joint_pos_ (%zu) does not match that of the robot model (%zu)",
                   joint_pos.size(), num_joints_);
      return false;
    }
    memcpy(q_.data.data(), joint_pos.data(), joint_pos.size()*sizeof(double));
    return true;
  }

  bool KDLKinematics::verify_link_name(const std::string &link_name) {
    if (link_name == root_name_) {
      return true;
    }
    if (link_name_map_.find(link_name) == link_name_map_.end()) {
      std::string links;
      for (auto i = 0u; i < chain_.getNrOfSegments(); i++) {
        links += "\n" + chain_.getSegment(i).getName();
      }
      RCLCPP_ERROR(LOGGER, "The link %s was not found in the robot chain. Available links are: %s",
                   link_name.c_str(), links.c_str());
      return false;
    }
    return true;
  }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kdl_plugin::KDLKinematics,
                       kinematics_interface::KinematicsBaseClass)