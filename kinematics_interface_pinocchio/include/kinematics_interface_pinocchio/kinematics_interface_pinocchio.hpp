// Copyright (c) 2024, Saif Sidhik.
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
/// \author: Saif Sidhik
/// \description: Pinocchio plugin for kinematics interface

#ifndef KINEMATICS_INTERFACE_PINOCCHIO__KINEMATICS_INTERFACE_PINOCCHIO_HPP_
#define KINEMATICS_INTERFACE_PINOCCHIO__KINEMATICS_INTERFACE_PINOCCHIO_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "kinematics_interface/kinematics_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace kinematics_interface_pinocchio
{
class KinematicsInterfacePinocchio : public kinematics_interface::KinematicsInterface
{
public:
  bool initialize(
    const std::string & robot_description,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string & param_namespace) override;

  bool convert_cartesian_deltas_to_joint_deltas(
    const Eigen::VectorXd & joint_pos, const Eigen::Matrix<double, 6, 1> & delta_x,
    const std::string & link_name, Eigen::VectorXd & delta_theta) override;

  bool convert_joint_deltas_to_cartesian_deltas(
    const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & delta_theta,
    const std::string & link_name, Eigen::Matrix<double, 6, 1> & delta_x) override;

  bool calculate_link_transform(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Isometry3d & transform) override;

  bool calculate_jacobian(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian) override;

  bool calculate_jacobian_inverse(
    const Eigen::VectorXd & joint_pos, const std::string & link_name,
    Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse) override;

private:
  // verification methods
  bool verify_initialized();
  bool verify_link_name(const std::string & link_name);
  bool verify_joint_vector(const Eigen::VectorXd & joint_vector);
  bool verify_jacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian);
  bool verify_jacobian_inverse(const Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse);
  bool verify_period(const double dt);

  bool initialized = false;
  std::string root_name_;
  Eigen::Index num_joints_;

  pinocchio::Model model_;
  std::shared_ptr<pinocchio::Data> data_;
  Eigen::VectorXd q_;
  Eigen::MatrixXd jacobian_;
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse_;
  Eigen::MatrixXd frame_tf_;

  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
  std::unordered_map<std::string, int> link_name_map_;
  double alpha;  // damping term for Jacobian inverse
  Eigen::MatrixXd I;
};

}  // namespace kinematics_interface_pinocchio

#endif  // KINEMATICS_INTERFACE_PINOCCHIO__KINEMATICS_INTERFACE_PINOCCHIO_HPP_
