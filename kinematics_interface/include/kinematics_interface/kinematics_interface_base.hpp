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

#ifndef IK_PLUGIN_BASE__IK_PLUGIN_BASE_HPP_
#define IK_PLUGIN_BASE__IK_PLUGIN_BASE_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace kinematics_interface
{
class KinematicsBaseClass
{
public:
    KinematicsBaseClass();
  ~KinematicsBaseClass();

  /**
   * \brief Create an object which takes Cartesian delta-x and converts to joint delta-theta.
   * It uses the Jacobian from MoveIt.
   */
  virtual bool
  initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string & group_name) = 0;

  /**
   * \brief Update the positions and velocities of the robot model
   * \param joint_pos joint positions of the robot in radians
   * \param joint_vel joint velocities of the robot in radians/sec
   * \return true if successful
   */
  virtual bool update_robot_state(const std::vector<double>& joint_pos, const std::vector<double>& joint_vel) = 0;

  /**
   * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
   * \param delta_x_vec input Cartesian deltas (x, y, z, rx, ry, rz)
   * \param control_frame_to_ik_base transform the requested delta_x to ik_base frame
   * \param delta_theta_vec output vector with joint states
   * \return true if successful
   */
  virtual bool
  convert_cartesian_deltas_to_joint_deltas(std::vector<double> & delta_x_vec, std::vector<double> & delta_theta_vec) = 0;

    /**
    * \brief Calculates the joint transform specified by segment name using the last set robot state.
    * \param transform_vec output vector with three element of the segment's position and 9 elements of
    * the segments rotation matrix in column major format.
     *\param segment_name the name of the segment to find the transform for
    * \return true if successful
    */
    virtual bool
    calculate_segment_transform(std::vector<double> & transform_vec, const std::string & segment_name) = 0;

  /**
   * \brief Convert joint delta-theta to Cartesian delta-x, using the Jacobian.
   * \param[in] delta_theta_vec vector with joint states
   * \param[in] tf_ik_base_to_desired_cartesian_frame transformation to the desired Cartesian frame. Use identity matrix to stay in the ik_base frame.
   * \param[out] delta_x_vec  Cartesian deltas (x, y, z, rx, ry, rz)
   * \return true if successful
   */
  virtual bool
  convert_joint_deltas_to_cartesian_deltas(std::vector<double> &  delta_theta_vec, std::vector<double> & delta_x_vec) = 0;

};

}  // namespace kinematics_interface

#endif  // IK_PLUGIN_BASE__IK_PLUGIN_BASE_HPP_
