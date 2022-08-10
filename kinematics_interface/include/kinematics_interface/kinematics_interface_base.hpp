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
/// \description: Base class for kinematics interface

#ifndef KINEMATICS_INTERFACE__KINEMATICS_INTERFACE_BASE_HPP_
#define KINEMATICS_INTERFACE__KINEMATICS_INTERFACE_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace kinematics_interface
{
class KinematicsBaseClass
{
public:
  KinematicsBaseClass() = default;

  virtual ~KinematicsBaseClass() = default;

  /**
     * \brief Create an interface object which takes calculate forward and inverse kinematics
     */
  virtual bool initialize(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & end_effector_name) = 0;

  /**
     * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
     * \param[in] joint_pos joint positions of the robot in radians
     * \param[in] delta_x_vec input Cartesian deltas (x, y, z, wx, wy, wz)
     * \param[out] delta_theta_vec output vector with joint states
     * \return true if successful
     */
  virtual bool convert_cartesian_deltas_to_joint_deltas(
    const std::vector<double> & joint_pos, const std::vector<double> & delta_x_vec,
    const std::string & link_name, std::vector<double> & delta_theta_vec) = 0;

  /**
     * \brief Convert joint delta-theta to Cartesian delta-x.
     * \param joint_pos joint positions of the robot in radiansgit checkout
     * \param[in] delta_theta_vec vector with joint states
     * \param[out] delta_x_vec  Cartesian deltas (x, y, z, wx, wy, wz)
     * \return true if successful
     */
  virtual bool convert_joint_deltas_to_cartesian_deltas(
    const std::vector<double> & joint_pos, const std::vector<double> & delta_theta_vec,
    const std::string & link_name, std::vector<double> & delta_x_vec) = 0;

  /**
    * \brief Calculates the joint transform for a specified link using provided joint positions.
    * \param[in] joint_pos joint positions of the robot in radians
    * \param[in] link_name the name of the link to find the transform for
    * \param[out] transform_vec transformation matrix of the specified link in column major format.
    * \return true if successful
    */
  virtual bool calculate_link_transform(
    const std::vector<double> & joint_pos, const std::string & link_name,
    std::vector<double> & transform_vec) = 0;

  /**
    * \brief Calculates the joint transform for a specified link using provided joint positions.
    * \param[in] joint_pos joint positions of the robot in radians
    * \param[in] link_name the name of the link to find the transform for
    * \param[out] jacobian Jacobian matrix of the specified link in column major format.
    * \return true if successful
    */
  virtual bool calculate_jacobian(
    const std::vector<double> & joint_pos, const std::string & link_name,
    std::vector<double> & jacobian) = 0;
};

}  // namespace kinematics_interface

#endif  // KINEMATICS_INTERFACE__KINEMATICS_INTERFACE_BASE_HPP_
