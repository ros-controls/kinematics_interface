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
/// \author: Paul Gesel, Christoph Froehlich

#ifndef KINEMATICS_INTERFACE_COMMON_TESTS_HPP_
#define KINEMATICS_INTERFACE_COMMON_TESTS_HPP_

#include <gmock/gmock.h>
#include <memory>
#include <string>
#include <vector>

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

MATCHER_P2(MatrixNear, expected, tol, "Two matrices are approximately equal")
{
  return arg.isApprox(expected, tol);
}

template <typename PluginUnderTest>
class TestPlugin : public ::testing::Test
{
public:
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> ik_loader_;
  std::shared_ptr<kinematics_interface::KinematicsInterface> ik_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::string end_effector_ = "link2";
  // base_link -> joint1 -> link1 -> joint2 -> link2 -> joint3 -> link3
  std::string urdf_ = std::string(ros2_control_test_assets::urdf_head) +
                      std::string(ros2_control_test_assets::urdf_tail);

  void SetUp()
  {
    // init ros
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
    std::string plugin_name = PluginUnderTest::Name();
    ik_loader_ =
      std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        "kinematics_interface", "kinematics_interface::KinematicsInterface");
    ik_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      ik_loader_->createUnmanagedInstance(plugin_name));

    node_->declare_parameter("verbose", true);
    node_->declare_parameter("robot_description", urdf_);
    node_->declare_parameter("tip", end_effector_);
    node_->declare_parameter("base", std::string(""));

    PluginUnderTest::set_custom_node_parameters(node_);
  }

  void TearDown()
  {
    node_->shutdown();
    // shutdown ros
    rclcpp::shutdown();
  }

  /**
   * \brief Used for testing initialization from parameters.
   * Elsewhere, `urdf_` member is used.
  */

  void loadURDFParameter(const std::string & urdf)
  {
    rclcpp::Parameter param("robot_description", urdf);
    node_->set_parameter(param);
  }

  /**
   * \brief Used for testing initialization from parameters.
   * Elsewhere, `end_effector_` member is used.
  */
  void loadTipParameter(const std::string & tip)
  {
    rclcpp::Parameter param("tip", tip);
    node_->set_parameter(param);
  }

  /**
   * \brief Used for testing initialization from parameters.
   * Elsewhere, `""` is used.
  */
  void loadBaseParameter(const std::string & base)
  {
    rclcpp::Parameter param("base", base);
    node_->set_parameter(param);
  }
};

TYPED_TEST_SUITE_P(TestPlugin);

TYPED_TEST_P(TestPlugin, basic_plugin_function)
{
  this->loadTipParameter("link3");

  // initialize the plugin
  ASSERT_TRUE(this->ik_->initialize(this->urdf_, this->node_->get_node_parameters_interface(), ""));

  // calculate end effector transform
  auto pos = Eigen::VectorXd::Zero(3);
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(
    this->ik_->calculate_link_transform(pos, this->end_effector_, end_effector_transform));

  // convert cartesian delta to joint delta
  kinematics_interface::Vector6d delta_x = kinematics_interface::Vector6d::Zero();
  delta_x[2] = 1;  // vz
  Eigen::VectorXd delta_theta = Eigen::VectorXd::Zero(3);
  ASSERT_TRUE(this->ik_->convert_cartesian_deltas_to_joint_deltas(
    pos, delta_x, this->end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  kinematics_interface::Vector6d delta_x_est;
  ASSERT_TRUE(this->ik_->convert_joint_deltas_to_cartesian_deltas(
    pos, delta_theta, this->end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  EXPECT_THAT(delta_x, MatrixNear(delta_x_est, 0.02));

  // calculate jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, 3>::Zero();
  ASSERT_TRUE(this->ik_->calculate_jacobian(pos, this->end_effector_, jacobian));

  // calculate jacobian inverse
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse =
    jacobian.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse_est =
    Eigen::Matrix<double, 3, 6>::Zero();
  ASSERT_TRUE(
    this->ik_->calculate_jacobian_inverse(pos, this->end_effector_, jacobian_inverse_est));

  // ensure jacobian inverse math is correct
  EXPECT_THAT(jacobian_inverse, MatrixNear(jacobian_inverse_est, 0.02));
}

TYPED_TEST_P(TestPlugin, plugin_function_reduced_model_tip)
{
  // initialize the plugin
  ASSERT_TRUE(this->ik_->initialize(this->urdf_, this->node_->get_node_parameters_interface(), ""));

  // calculate end effector transform
  auto pos = Eigen::VectorXd::Zero(2);
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(
    this->ik_->calculate_link_transform(pos, this->end_effector_, end_effector_transform));

  // convert cartesian delta to joint delta
  kinematics_interface::Vector6d delta_x = kinematics_interface::Vector6d::Zero();
  delta_x[2] = 1;  // vz
  Eigen::VectorXd delta_theta = Eigen::VectorXd::Zero(2);
  ASSERT_TRUE(this->ik_->convert_cartesian_deltas_to_joint_deltas(
    pos, delta_x, this->end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  kinematics_interface::Vector6d delta_x_est;
  ASSERT_TRUE(this->ik_->convert_joint_deltas_to_cartesian_deltas(
    pos, delta_theta, this->end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  EXPECT_THAT(delta_x, MatrixNear(delta_x_est, 0.02));

  // calculate jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, 2>::Zero();
  ASSERT_TRUE(this->ik_->calculate_jacobian(pos, this->end_effector_, jacobian));

  // calculate jacobian inverse
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse =
    jacobian.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse_est =
    Eigen::Matrix<double, 2, 6>::Zero();
  ASSERT_TRUE(
    this->ik_->calculate_jacobian_inverse(pos, this->end_effector_, jacobian_inverse_est));

  // ensure jacobian inverse math is correct
  EXPECT_THAT(jacobian_inverse, MatrixNear(jacobian_inverse_est, 0.02));
}

TYPED_TEST_P(TestPlugin, plugin_function_reduced_model_base)
{
  this->loadTipParameter("link3");
  this->loadBaseParameter("link1");

  // initialize the plugin
  ASSERT_TRUE(this->ik_->initialize(this->urdf_, this->node_->get_node_parameters_interface(), ""));

  // calculate end effector transform
  auto pos = Eigen::VectorXd::Zero(2);
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(
    this->ik_->calculate_link_transform(pos, this->end_effector_, end_effector_transform));

  // convert cartesian delta to joint delta
  kinematics_interface::Vector6d delta_x = kinematics_interface::Vector6d::Zero();
  delta_x[2] = 1;  // vz
  Eigen::VectorXd delta_theta = Eigen::VectorXd::Zero(2);
  ASSERT_TRUE(this->ik_->convert_cartesian_deltas_to_joint_deltas(
    pos, delta_x, this->end_effector_, delta_theta));
  // jacobian inverse for vz is singular in this configuration
  EXPECT_THAT(delta_theta, MatrixNear(Eigen::Vector2d::Zero(), 0.02));

  // convert joint delta to cartesian delta
  kinematics_interface::Vector6d delta_x_est;
  ASSERT_TRUE(this->ik_->convert_joint_deltas_to_cartesian_deltas(
    pos, delta_theta, this->end_effector_, delta_x_est));
  // joint deltas from zero should produce zero cartesian deltas
  EXPECT_THAT(delta_x_est, MatrixNear(kinematics_interface::Vector6d::Zero(), 0.02));

  // calculate jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, 2>::Zero();
  ASSERT_TRUE(this->ik_->calculate_jacobian(pos, this->end_effector_, jacobian));

  // calculate jacobian inverse
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse =
    jacobian.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse_est =
    Eigen::Matrix<double, 2, 6>::Zero();
  ASSERT_TRUE(
    this->ik_->calculate_jacobian_inverse(pos, this->end_effector_, jacobian_inverse_est));

  // ensure jacobian inverse math is correct
  EXPECT_THAT(jacobian_inverse, MatrixNear(jacobian_inverse_est, 0.02));
}

TYPED_TEST_P(TestPlugin, plugin_function_std_vector)
{
  // initialize the plugin
  ASSERT_TRUE(this->ik_->initialize(this->urdf_, this->node_->get_node_parameters_interface(), ""));

  // calculate end effector transform
  std::vector<double> pos = {0, 0};
  Eigen::Isometry3d end_effector_transform;
  ASSERT_TRUE(
    this->ik_->calculate_link_transform(pos, this->end_effector_, end_effector_transform));

  // convert cartesian delta to joint delta
  std::vector<double> delta_x = {0, 0, 0, 0, 0, 0};
  delta_x[2] = 1;
  std::vector<double> delta_theta = {0, 0};
  ASSERT_TRUE(this->ik_->convert_cartesian_deltas_to_joint_deltas(
    pos, delta_x, this->end_effector_, delta_theta));

  // convert joint delta to cartesian delta
  std::vector<double> delta_x_est(6);
  ASSERT_TRUE(this->ik_->convert_joint_deltas_to_cartesian_deltas(
    pos, delta_theta, this->end_effector_, delta_x_est));

  // Ensure kinematics math is correct
  EXPECT_THAT(delta_x, ::testing::Pointwise(::testing::DoubleNear(0.02), delta_x_est));

  // calculate jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, 2>::Zero();
  ASSERT_TRUE(this->ik_->calculate_jacobian(pos, this->end_effector_, jacobian));

  // calculate jacobian inverse
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse =
    jacobian.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_inverse_est =
    Eigen::Matrix<double, 2, 6>::Zero();
  ASSERT_TRUE(
    this->ik_->calculate_jacobian_inverse(pos, this->end_effector_, jacobian_inverse_est));

  // ensure jacobian inverse math is correct
  EXPECT_THAT(jacobian_inverse, MatrixNear(jacobian_inverse_est, 0.02));
}

TYPED_TEST_P(TestPlugin, plugin_calculate_frame_difference)
{
  // compute the difference between two cartesian frames
  Eigen::Matrix<double, 7, 1> x_a, x_b;
  x_a << 0, 1, 0, 0, 0, 0, 1;
  x_b << 2, 3, 0, 0, 1, 0, 0;
  double dt = 1.0;
  kinematics_interface::Vector6d delta_x = kinematics_interface::Vector6d::Zero();
  kinematics_interface::Vector6d delta_x_est;
  delta_x_est << 2, 2, 0, 0, 3.14, 0;
  ASSERT_TRUE(this->ik_->calculate_frame_difference(x_a, x_b, dt, delta_x));

  // ensure that difference math is correct
  EXPECT_THAT(delta_x, MatrixNear(delta_x_est, 0.02));
}

TYPED_TEST_P(TestPlugin, plugin_calculate_frame_difference_std_vector)
{
  // compute the difference between two cartesian frames
  std::vector<double> x_a(7), x_b(7);
  x_a = {0, 1, 0, 0, 0, 0, 1};
  x_b = {2, 3, 0, 0, 1, 0, 0};
  double dt = 1.0;
  std::vector<double> delta_x = {0, 0, 0, 0, 0, 0};
  std::vector<double> delta_x_est = {2, 2, 0, 0, 3.14, 0};
  ASSERT_TRUE(this->ik_->calculate_frame_difference(x_a, x_b, dt, delta_x));

  // ensure that difference math is correct
  EXPECT_THAT(delta_x, ::testing::Pointwise(::testing::DoubleNear(0.02), delta_x_est));
}

TYPED_TEST_P(TestPlugin, incorrect_input_sizes)
{
  // initialize the plugin
  ASSERT_TRUE(this->ik_->initialize(this->urdf_, this->node_->get_node_parameters_interface(), ""));

  // define correct values
  auto pos = Eigen::VectorXd::Zero(2);
  Eigen::Isometry3d end_effector_transform;
  kinematics_interface::Vector6d delta_x = kinematics_interface::Vector6d::Zero();
  delta_x[2] = 1;
  Eigen::VectorXd delta_theta = Eigen::VectorXd::Zero(2);
  kinematics_interface::Vector6d delta_x_est;
  Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian = Eigen::Matrix<double, 2, 6>::Zero();
  Eigen::Matrix<double, 7, 1> x_a, x_b;

  // wrong size input vector
  Eigen::VectorXd vec_5 = Eigen::VectorXd::Zero(5);

  // wrong size input jacobian
  Eigen::Matrix<double, Eigen::Dynamic, 6> mat_5_6 = Eigen::Matrix<double, 5, 6>::Zero();

  // wrong value for period
  double dt = -0.1;

  // calculate transform
  ASSERT_FALSE(
    this->ik_->calculate_link_transform(vec_5, this->end_effector_, end_effector_transform));
  ASSERT_FALSE(
    this->ik_->calculate_link_transform(pos, "link_not_in_model", end_effector_transform));

  // convert cartesian delta to joint delta
  ASSERT_FALSE(this->ik_->convert_cartesian_deltas_to_joint_deltas(
    vec_5, delta_x, this->end_effector_, delta_theta));
  ASSERT_FALSE(this->ik_->convert_cartesian_deltas_to_joint_deltas(
    pos, delta_x, "link_not_in_model", delta_theta));
  ASSERT_FALSE(
    this->ik_->convert_cartesian_deltas_to_joint_deltas(pos, delta_x, this->end_effector_, vec_5));

  // convert joint delta to cartesian delta
  ASSERT_FALSE(this->ik_->convert_joint_deltas_to_cartesian_deltas(
    vec_5, delta_theta, this->end_effector_, delta_x_est));
  ASSERT_FALSE(this->ik_->convert_joint_deltas_to_cartesian_deltas(
    pos, vec_5, this->end_effector_, delta_x_est));
  ASSERT_FALSE(this->ik_->convert_joint_deltas_to_cartesian_deltas(
    pos, delta_theta, "link_not_in_model", delta_x_est));

  // calculate jacobian inverse
  ASSERT_FALSE(this->ik_->calculate_jacobian_inverse(vec_5, this->end_effector_, jacobian));
  ASSERT_FALSE(this->ik_->calculate_jacobian_inverse(pos, this->end_effector_, mat_5_6));
  ASSERT_FALSE(this->ik_->calculate_jacobian_inverse(pos, "link_not_in_model", jacobian));

  // compute the difference between two cartesian frames
  ASSERT_FALSE(this->ik_->calculate_frame_difference(x_a, x_b, dt, delta_x));
}

TYPED_TEST_P(TestPlugin, plugin_no_robot_description)
{
  this->loadURDFParameter("");
  ASSERT_FALSE(this->ik_->initialize("", this->node_->get_node_parameters_interface(), ""));
}

TYPED_TEST_P(TestPlugin, plugin_no_parameter_tip)
{
  this->loadTipParameter("");
  ASSERT_FALSE(
    this->ik_->initialize(this->urdf_, this->node_->get_node_parameters_interface(), ""));
}

REGISTER_TYPED_TEST_SUITE_P(
  TestPlugin, basic_plugin_function, plugin_function_reduced_model_tip,
  plugin_function_reduced_model_base, plugin_function_std_vector, plugin_calculate_frame_difference,
  plugin_calculate_frame_difference_std_vector, incorrect_input_sizes, plugin_no_robot_description,
  plugin_no_parameter_tip);

#endif  // KINEMATICS_INTERFACE_COMMON_TESTS_HPP_
