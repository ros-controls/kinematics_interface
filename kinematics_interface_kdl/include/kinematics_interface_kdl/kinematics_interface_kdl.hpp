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
/// \description: KDL plugin for kinematics interface

#pragma once

#include <kdl/frames.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/treejnttojacsolver.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "tf2_eigen_kdl/tf2_eigen_kdl.hpp"

namespace kinematics_interface_kdl
{
class KDLKinematics : public kinematics_interface::KinematicsBaseClass
{
public:
  /**
     * \brief Initialize plugin. This method must be called before any other.
     */
  bool initialize(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & end_effector_name) override;

  /**
     * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
     * \param[in] joint_pos joint positions of the robot in radians
     * \param[in] delta_x input Cartesian deltas (x, y, z, wx, wy, wz)
     * \param[in] link_name the link name at which delta_x is applied
     * \param[out] delta_theta outputs joint deltas
     * \return true if successful
     */
  bool convert_cartesian_deltas_to_joint_deltas(
    const Eigen::VectorXd & joint_pos, const Eigen::Matrix<double, 6, 1> & delta_x,
    const std::string & link_name, Eigen::VectorXd & delta_theta) override;

  /**
     * \brief Convert joint delta-theta to Cartesian delta-x.
     * \param joint_pos joint positions of the robot in radians
     * \param[in] delta_theta joint deltas
     * \param[in] link_name the link name at which delta_x is calculated
     * \param[out] delta_x  Cartesian deltas (x, y, z, wx, wy, wz)
     * \return true if successful
     */
  bool convert_joint_deltas_to_cartesian_deltas(
    const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & delta_theta,
    const std::string & link_name, Eigen::Matrix<double, 6, 1> & delta_x) override;

  /**
    * \brief Calculates the joint transform for a specified link using provided joint positions.
    * \param[in] joint_pos joint positions of the robot in radians
    * \param[in] link_name the name of the link to find the transform for
    * \param[out] transform transformation matrix of the specified link
    * \return true if successful
    */
  bool calculate_link_transform(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Isometry3d & transform) override;

  /**
    * \brief Calculates the jacobian for a specified link using provided joint positions.
    * \param[in] joint_pos joint positions of the robot in radians
    * \param[in] link_name the name of the link to find the transform for
    * \param[out] jacobian Jacobian matrix of the specified link in column major format.
    * \return true if successful
    */
  bool calculate_jacobian(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian) override;

private:
  //verification methods
  bool verify_initialized();
  bool verify_link_name(const std::string & link_name);
  bool verify_joint_vector(const Eigen::VectorXd & joint_vector);
  bool verify_jacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian);

  bool initialized = false;
  std::string root_name_;
  size_t num_joints_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  KDL::JntArray q_;
  KDL::Frame frame_;
  std::shared_ptr<KDL::Jacobian> jacobian_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unordered_map<std::string, int> link_name_map_;
  double alpha;  // damping term for Jacobian inverse
  Eigen::MatrixXd I;
};

}  // namespace kinematics_interface_kdl
