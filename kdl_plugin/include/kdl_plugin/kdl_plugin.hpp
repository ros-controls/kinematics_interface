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

#include "kinematics_interface/kinematics_interface_base.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/treejnttojacsolver.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"


namespace kdl_plugin
{

    class KDLKinematics : public kinematics_interface::KinematicsBaseClass
    {
    public:
        /**
         * \brief KDL implementation of ros2_control kinematics interface
         */
        virtual bool
        initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string &end_effector_name);

      /**
       * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
       * \param[in] joint_pos joint positions of the robot in radians
       * \param[in] delta_x_vec input Cartesian deltas (x, y, z, wx, wy, wz)
       * \param[out] delta_theta_vec output vector with joint states
       * \return true if successful
       */
      virtual bool
      convert_cartesian_deltas_to_joint_deltas(const std::vector<double> &joint_pos,
                                               const std::vector<double> &delta_x_vec,
                                               std::vector<double> &delta_theta_vec);

      /**
       * \brief Convert joint delta-theta to Cartesian delta-x.
       * \param joint_pos joint positions of the robot in radians
       * \param[in] delta_theta_vec vector with joint states
       * \param[out] delta_x_vec  Cartesian deltas (x, y, z, wx, wy, wz)
       * \return true if successful
       */
      virtual bool
      convert_joint_deltas_to_cartesian_deltas(const std::vector<double> &joint_pos,
                                               const std::vector<double> &delta_theta_vec,
                                               std::vector<double> &delta_x_vec);

      /**
      * \brief Calculates the joint transform for a specified link using provided joint positions.
      * \param[in] joint_pos joint positions of the robot in radians
      * \param[in] link_name the name of the link to find the transform for
      * \param[out] transform_vec transformation matrix of the specified link in column major format.
      * \return true if successful
      */
      virtual bool
      calculate_link_transform(const std::vector<double> &joint_pos, const std::string &link_name,
                               std::vector<double> &transform_vec);


    private:
        bool update_joint_array(const std::vector<double>& joint_pos);

        bool initialized = false;
        std::string end_effector_name_;
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
        double alpha; // damping term for Jacobian inverse
        Eigen::Matrix<double,6,1> delta_x;
        Eigen::VectorXd delta_theta;
        Eigen::MatrixXd I;
    };

}  // namespace moveit_differential_ik_plugin
