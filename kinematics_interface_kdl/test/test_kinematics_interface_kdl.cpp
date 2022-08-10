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
/// \author: Paul Gesel

#include <gmock/gmock.h>
#include <memory>
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "pluginlib/class_loader.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestKDLPlugin : public ::testing::Test
{
public:
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>> ik_loader_;
  std::shared_ptr<kinematics_interface::KinematicsBaseClass> ik_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::string end_effector_ = "link2";

  void SetUp()
  {
    // init ros
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
    std::string plugin_name = "kinematics_interface_kdl/KDLKinematics";
    ik_loader_ =
      std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>>(
        "kinematics_interface", "kinematics_interface::KinematicsBaseClass");
    ik_ = std::unique_ptr<kinematics_interface::KinematicsBaseClass>(
      ik_loader_->createUnmanagedInstance(plugin_name));
  }

  void TearDown()
  {
    // shutdown ros
    rclcpp::shutdown();
  }

  void loadURDFParameter()
  {
    auto urdf = std::string(ros2_control_test_assets::urdf_head) +
                std::string(ros2_control_test_assets::urdf_tail);
    rclcpp::Parameter param("robot_description", urdf);
    node_->declare_parameter("robot_description", "");
    node_->set_parameter(param);
  }

  void loadAlphaParameter()
  {
    rclcpp::Parameter param("alpha", 0.005);
    node_->declare_parameter("alpha", 0.005);
    node_->set_parameter(param);
  }
};

TEST_F(TestKDLPlugin, KDL_plugin_function)
{
  // load robot description and alpha to parameter server
  loadURDFParameter();
  loadAlphaParameter();

  // initialize the  plugin
  ASSERT_TRUE(ik_->initialize(node_, end_effector_));

  // calculate end effector transform
  std::vector<double> pos = {0, 0};
  std::vector<double> end_effector_transform(16);
  ASSERT_TRUE(ik_->calculate_link_transform(pos, end_effector_, end_effector_transform));

  // convert cartesian delta to joint delta
  std::vector<double> delta_x = {0, 0, .5, 0, 0, 0};
  std::vector<double> delta_theta(2);
  ASSERT_TRUE(
    ik_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  std::vector<double> delta_x_est(6);
  ASSERT_TRUE(
    ik_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  for (auto i = 0ul; i < delta_x.size(); i++)
  {
    ASSERT_NEAR(delta_x[i], delta_x_est[i], 0.01);
  }
}

TEST_F(TestKDLPlugin, incorrect_input_sizes)
{
  // load robot description and alpha to parameter server
  loadURDFParameter();
  loadAlphaParameter();

  // initialize the  plugin
  ASSERT_TRUE(ik_->initialize(node_, end_effector_));

  // define correct values
  std::vector<double> pos = {0, 0};
  std::vector<double> end_effector_transform(16);
  std::vector<double> delta_x = {0, 0, 1, 0, 0, 0};
  std::vector<double> delta_theta(2);
  std::vector<double> delta_x_est(6);

  // wrong size input vector
  std::vector<double> vec_5 = {1.0, 2.0, 3.0, 4.0, 5.0};

  // calculate transform
  ASSERT_FALSE(ik_->calculate_link_transform(vec_5, end_effector_, end_effector_transform));
  ASSERT_FALSE(ik_->calculate_link_transform(pos, "link_not_in_model", end_effector_transform));
  ASSERT_FALSE(ik_->calculate_link_transform(pos, end_effector_, vec_5));

  // convert cartesian delta to joint delta
  ASSERT_FALSE(
    ik_->convert_cartesian_deltas_to_joint_deltas(vec_5, delta_x, end_effector_, delta_theta));
  ASSERT_FALSE(
    ik_->convert_cartesian_deltas_to_joint_deltas(pos, vec_5, end_effector_, delta_theta));
  ASSERT_FALSE(
    ik_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, "link_not_in_model", delta_theta));
  ASSERT_FALSE(ik_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, end_effector_, vec_5));

  // convert joint delta to cartesian delta
  ASSERT_FALSE(
    ik_->convert_joint_deltas_to_cartesian_deltas(vec_5, delta_theta, end_effector_, delta_x_est));
  ASSERT_FALSE(
    ik_->convert_joint_deltas_to_cartesian_deltas(pos, vec_5, end_effector_, delta_x_est));
  ASSERT_FALSE(ik_->convert_joint_deltas_to_cartesian_deltas(
    pos, delta_theta, "link_not_in_model", delta_x_est));
  ASSERT_FALSE(
    ik_->convert_joint_deltas_to_cartesian_deltas(pos, delta_theta, end_effector_, vec_5));
}

TEST_F(TestKDLPlugin, KDL_plugin_no_robot_description)
{
  // load alpha to parameter server
  loadAlphaParameter();
  ASSERT_FALSE(ik_->initialize(node_, end_effector_));
}
