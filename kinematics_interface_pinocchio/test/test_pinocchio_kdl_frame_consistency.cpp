// Copyright (c) 2026, Saif Sidhik.
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
/// \description: Regression tests that the Pinocchio plugin expresses the Jacobian and
///              link transforms in the chain-root frame, matching the KDL reference plugin
///              (see https://github.com/ros-controls/kinematics_interface/issues/256).

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Geometry"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
constexpr double kTol = 1e-4;
const char kPinocchioPlugin[] = "kinematics_interface_pinocchio/KinematicsInterfacePinocchio";
const char kKdlPlugin[] = "kinematics_interface_kdl/KinematicsInterfaceKDL";
}  // namespace

// These tests load BOTH plugins on the same robot and assert the Pinocchio results match the KDL
// reference, for both the default base (chain root == universe) and a custom base (chain root
// rotated with respect to universe).
class PinocchioPluginFrameTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");
  }

  void TearDown() override
  {
    instances_.clear();
    nodes_.clear();
    loader_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<kinematics_interface::KinematicsInterface> make_plugin(
    const std::string & plugin_name, const std::string & base, const std::string & tip,
    const std::string & node_name)
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name);
    node->declare_parameter("robot_description", urdf_);
    node->declare_parameter("tip", tip);
    node->declare_parameter("base", base);
    node->declare_parameter("alpha", 0.005);
    nodes_.push_back(node);

    auto ik = std::shared_ptr<kinematics_interface::KinematicsInterface>(
      loader_->createUnmanagedInstance(plugin_name));
    if (!ik->initialize(urdf_, node->get_node_parameters_interface(), ""))
    {
      return nullptr;
    }
    instances_.push_back(ik);
    return ik;
  }

  // Assert that the Pinocchio Jacobian and link transform match the KDL reference plugin at `q`.
  void expect_matches_kdl(
    const std::string & base, const std::string & tip, const Eigen::VectorXd & q,
    const std::string & tag)
  {
    auto kdl = make_plugin(kKdlPlugin, base, tip, "kdl_" + tag);
    auto pin = make_plugin(kPinocchioPlugin, base, tip, "pin_" + tag);
    ASSERT_NE(kdl, nullptr) << "KDL plugin failed to initialize for case '" << tag << "'";
    ASSERT_NE(pin, nullptr) << "Pinocchio plugin failed to initialize for case '" << tag << "'";

    const Eigen::Index dof = q.size();
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_kdl(6, dof);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_pin(6, dof);
    jacobian_kdl.setZero();
    jacobian_pin.setZero();
    ASSERT_TRUE(kdl->calculate_jacobian(q, tip, jacobian_kdl));
    ASSERT_TRUE(pin->calculate_jacobian(q, tip, jacobian_pin));
    EXPECT_TRUE(jacobian_pin.isApprox(jacobian_kdl, kTol))
      << "[" << tag << "] Jacobian mismatch.\nPinocchio:\n"
      << jacobian_pin << "\nKDL:\n"
      << jacobian_kdl;

    Eigen::Isometry3d transform_kdl;
    Eigen::Isometry3d transform_pin;
    ASSERT_TRUE(kdl->calculate_link_transform(q, tip, transform_kdl));
    ASSERT_TRUE(pin->calculate_link_transform(q, tip, transform_pin));
    EXPECT_TRUE(transform_pin.isApprox(transform_kdl, kTol))
      << "[" << tag << "] link transform mismatch.\nPinocchio:\n"
      << transform_pin.matrix() << "\nKDL:\n"
      << transform_kdl.matrix();
  }

  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
  std::vector<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> nodes_;
  std::vector<std::shared_ptr<kinematics_interface::KinematicsInterface>> instances_;
  // world -> base_joint(fixed) -> base_link
  //   -> joint1 -> link1 -> joint2 -> link2 -> joint3 -> link3
  std::string urdf_ = std::string(ros2_control_test_assets::urdf_head) +
                      std::string(ros2_control_test_assets::urdf_tail);
};

// Default base (empty `base`, chain root == "world" == universe, 3 DOF).
TEST_F(PinocchioPluginFrameTest, jacobian_and_transform_match_kdl_default_base)
{
  Eigen::VectorXd q(3);
  q << 0.5, -0.3, 0.8;
  expect_matches_kdl("", "link3", q, "default_base");
}

// Custom base ("link1", chain root rotated with respect to universe, 2 DOF).
// This is the configuration that a LOCAL_WORLD_ALIGNED-only fix gets wrong.
TEST_F(PinocchioPluginFrameTest, jacobian_and_transform_match_kdl_custom_base)
{
  Eigen::VectorXd q(2);
  q << 0.5, -0.3;
  expect_matches_kdl("link1", "link3", q, "custom_base");
}

// Hardcoded ground-truth oracle from issue #256 (KDL reference, default base, tip "link3",
// q = [0.5, -0.3, 0.8]). Independent of the KDL plugin being available at runtime.
TEST_F(PinocchioPluginFrameTest, pinocchio_jacobian_matches_hardcoded_kdl_ground_truth)
{
  Eigen::VectorXd q(3);
  q << 0.5, -0.3, 0.8;
  auto pin = make_plugin(kPinocchioPlugin, "", "link3", "pin_anchor");
  ASSERT_NE(pin, nullptr);

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, 3);
  jacobian.setZero();
  ASSERT_TRUE(pin->calculate_jacobian(q, "link3", jacobian));

  Eigen::Matrix<double, 6, 3> expected;
  expected << 0.0, 0.0, 0.0,   // vx
    -1.31354, -0.88206, 0.0,   // vy
    0.611022, -0.178802, 0.0,  // vz
    1.0, 1.0, 1.0,             // wx
    0.0, 0.0, 0.0,             // wy
    0.0, 0.0, 0.0;             // wz
  EXPECT_TRUE(jacobian.isApprox(expected, kTol)) << "Pinocchio Jacobian:\n" << jacobian;
}
