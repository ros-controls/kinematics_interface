// Copyright (c) 2026 b»robotized
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

#include <gmock/gmock.h>

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "kinematics_interface_ikfast/ikfast.h"
#include "kinematics_interface_ikfast/kinematics_interface_ikfast.hpp"
#include "rclcpp/rclcpp.hpp"

// 2-DOF planar robot: two revolute joints around Z, L1 = L2 = 0.5 m
//
//   base_link --[joint1, Z]--> link1 --(0.5,0,0)--[joint2, Z]--> link2 --(0.5,0,0)--> flange
//
// FK:  x = 0.5*cos(q1) + 0.5*cos(q1+q2)
//      y = 0.5*sin(q1) + 0.5*sin(q1+q2)
//      R = Rz(q1+q2)
//
// IK:  c2 = (x²+y² - 0.5) / 0.5,  two solutions ±s2

static const std::string URDF_2DOF = R"(
<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1"/>
  </joint>
  <link name="link1">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1"/>
  </joint>
  <link name="link2">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="ee_joint" type="fixed">
    <parent link="link2"/>
    <child link="flange"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
  </joint>
  <link name="flange"/>
</robot>
)";

namespace
{
static constexpr double L1 = 0.5;
static constexpr double L2 = 0.5;
}  // namespace

class TwoDofPlanarIKFast : public kinematics_interface_ikfast::KinematicsInterfaceIKFast
{
public:
  int get_num_joints_internal() override { return 2; }

  void compute_fk(const double * j, double * eetrans, double * eerot) override
  {
    const double c12 = std::cos(j[0] + j[1]);
    const double s12 = std::sin(j[0] + j[1]);

    eetrans[0] = L1 * std::cos(j[0]) + L2 * c12;
    eetrans[1] = L1 * std::sin(j[0]) + L2 * s12;
    eetrans[2] = 0.0;

    // Row-major rotation matrix: Rz(q1+q2)
    eerot[0] = c12;   eerot[1] = -s12;  eerot[2] = 0.0;
    eerot[3] = s12;   eerot[4] =  c12;  eerot[5] = 0.0;
    eerot[6] = 0.0;   eerot[7] =  0.0;  eerot[8] = 1.0;
  }

  void compute_ik(
    const double * etrans, const double * /*erot*/, const double * /*free*/,
    void * solutions_ptr) override
  {
    auto & sol_list = *reinterpret_cast<ikfast::IkSolutionList<double> *>(solutions_ptr);

    const double x = etrans[0];
    const double y = etrans[1];
    const double c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    if (std::fabs(c2) > 1.0)
    {
      return;  // target unreachable
    }

    const double s2_abs = std::sqrt(1.0 - c2 * c2);
    for (const double s2 : {s2_abs, -s2_abs})
    {
      const double q2 = std::atan2(s2, c2);
      const double q1 = std::atan2(y, x) - std::atan2(L2 * s2, L1 + L2 * c2);

      std::vector<ikfast::IkSingleDOFSolutionBase<double>> vinfos(2);
      vinfos[0].foffset = q1;
      vinfos[0].indices[0] = 0;
      vinfos[1].foffset = q2;
      vinfos[1].indices[0] = 0;
      sol_list.AddSolution(vinfos, {});
    }
  }
};

class TestIKFast : public ::testing::Test
{
public:
  std::shared_ptr<TwoDofPlanarIKFast> ik_;
  std::shared_ptr<rclcpp::Node> node_;

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_ikfast_node");
    node_->declare_parameter("tip", std::string("flange"));
    node_->declare_parameter("base", std::string("base_link"));
    node_->declare_parameter("alpha", 0.000005);
    ik_ = std::make_shared<TwoDofPlanarIKFast>();
  }

  bool initialize() { return ik_->initialize(URDF_2DOF, node_->get_node_parameters_interface(), ""); }
};

TEST_F(TestIKFast, initialize_succeeds)
{
  ASSERT_TRUE(initialize());
}

TEST_F(TestIKFast, initialize_fails_on_empty_urdf)
{
  ASSERT_FALSE(ik_->initialize("", node_->get_node_parameters_interface(), ""));
}

TEST_F(TestIKFast, calculate_link_transform_home_pose)
{
  // q = (0, 0)  →  EE at (1.0, 0.0, 0.0), identity rotation
  ASSERT_TRUE(initialize());
  Eigen::VectorXd q(2);
  q << 0.0, 0.0;
  Eigen::Isometry3d T;
  ASSERT_TRUE(ik_->calculate_link_transform(q, "flange", T));
  EXPECT_NEAR(T.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(T.translation().y(), 0.0, 1e-6);
  EXPECT_NEAR(T.translation().z(), 0.0, 1e-6);
  EXPECT_NEAR(T.linear()(0, 0), 1.0, 1e-6);
}

TEST_F(TestIKFast, calculate_link_transform_ninety_degrees)
{
  // q = (pi/2, 0)  →  EE at (0.0, 1.0, 0.0)
  ASSERT_TRUE(initialize());
  Eigen::VectorXd q(2);
  q << M_PI / 2.0, 0.0;
  Eigen::Isometry3d T;
  ASSERT_TRUE(ik_->calculate_link_transform(q, "flange", T));
  EXPECT_NEAR(T.translation().x(), 0.0, 1e-6);
  EXPECT_NEAR(T.translation().y(), 1.0, 1e-6);
}

TEST_F(TestIKFast, calculate_link_transform_wrong_link_fails)
{
  ASSERT_TRUE(initialize());
  Eigen::VectorXd q(2);
  q << 0.0, 0.0;
  Eigen::Isometry3d T;
  EXPECT_FALSE(ik_->calculate_link_transform(q, "link1", T));
}

TEST_F(TestIKFast, possible_joint_states_returns_two_solutions)
{
  ASSERT_TRUE(initialize());

  // Compute a target pose via FK, then IK must recover exactly 2 solutions (elbow-up/-down)
  Eigen::VectorXd q(2);
  q << 0.3, 0.5;
  Eigen::Isometry3d pose;
  ASSERT_TRUE(ik_->calculate_link_transform(q, "flange", pose));

  std::vector<std::vector<double>> solutions;
  ASSERT_TRUE(ik_->convert_cartesian_pose_to_possible_joint_states(pose, solutions));
  ASSERT_EQ(solutions.size(), 2u);

  // Both solutions must reproduce the same EE position
  for (const auto & sol : solutions)
  {
    Eigen::VectorXd q_sol = Eigen::Map<const Eigen::VectorXd>(sol.data(), sol.size());
    Eigen::Isometry3d check;
    ASSERT_TRUE(ik_->calculate_link_transform(q_sol, "flange", check));
    EXPECT_NEAR(check.translation().x(), pose.translation().x(), 1e-5);
    EXPECT_NEAR(check.translation().y(), pose.translation().y(), 1e-5);
  }
}

TEST_F(TestIKFast, possible_joint_states_unreachable_target)
{
  ASSERT_TRUE(initialize());

  // Target beyond reach (L1+L2 = 1.0 m, place at 2.0 m)
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << 2.0, 0.0, 0.0;

  std::vector<std::vector<double>> solutions;
  ASSERT_TRUE(ik_->convert_cartesian_pose_to_possible_joint_states(pose, solutions));
  EXPECT_TRUE(solutions.empty());
}

TEST_F(TestIKFast, closest_joint_state_picks_nearest_solution)
{
  ASSERT_TRUE(initialize());

  const std::vector<double> q_in = {0.3, 0.5};
  Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q_in.data(), q_in.size());
  Eigen::Isometry3d pose;
  ASSERT_TRUE(ik_->calculate_link_transform(q_eigen, "flange", pose));

  std::vector<double> result;
  ASSERT_TRUE(ik_->convert_cartesian_pose_to_closest_joint_state(pose, q_in, result));
  ASSERT_EQ(result.size(), 2u);

  // The returned solution must reproduce the target EE position
  Eigen::VectorXd result_eigen = Eigen::Map<const Eigen::VectorXd>(result.data(), result.size());
  Eigen::Isometry3d check;
  ASSERT_TRUE(ik_->calculate_link_transform(result_eigen, "flange", check));
  EXPECT_NEAR(check.translation().x(), pose.translation().x(), 1e-5);
  EXPECT_NEAR(check.translation().y(), pose.translation().y(), 1e-5);

  // And it should be the solution closest to q_in (elbow-up matches exactly)
  EXPECT_NEAR(result[0], q_in[0], 1e-5);
  EXPECT_NEAR(result[1], q_in[1], 1e-5);
}

TEST_F(TestIKFast, closest_joint_state_size_mismatch_fails)
{
  ASSERT_TRUE(initialize());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  std::vector<double> result;
  EXPECT_FALSE(ik_->convert_cartesian_pose_to_closest_joint_state(
    pose, std::vector<double>{0.0, 0.0, 0.0}, result));
}

TEST_F(TestIKFast, joint_state_within_range_valid_range)
{
  ASSERT_TRUE(initialize());

  const std::vector<double> q_in = {0.3, 0.5};
  Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q_in.data(), q_in.size());
  Eigen::Isometry3d pose;
  ASSERT_TRUE(ik_->calculate_link_transform(q_eigen, "flange", pose));

  std::vector<std::pair<double, double>> ranges = {{-M_PI, M_PI}, {-M_PI, M_PI}};
  std::vector<double> result;
  ASSERT_TRUE(ik_->convert_cartesian_pose_to_joint_state_within_range(pose, ranges, result));
  ASSERT_EQ(result.size(), 2u);

  // Result must be within the specified ranges
  EXPECT_GE(result[0], -M_PI);
  EXPECT_LE(result[0], M_PI);
  EXPECT_GE(result[1], -M_PI);
  EXPECT_LE(result[1], M_PI);
}

TEST_F(TestIKFast, joint_state_within_range_tight_range_excludes_all)
{
  ASSERT_TRUE(initialize());

  // q = (0.3, 0.5) → solutions near (0.3, 0.5) and (~0.8, -0.5)
  // Set a tiny range that excludes both
  const std::vector<double> q_in = {0.3, 0.5};
  Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q_in.data(), q_in.size());
  Eigen::Isometry3d pose;
  ASSERT_TRUE(ik_->calculate_link_transform(q_eigen, "flange", pose));

  // Range centred far from any solution (e.g. 2.5 ± 0.01)
  std::vector<std::pair<double, double>> ranges = {{2.49, 2.51}, {2.49, 2.51}};
  std::vector<double> result;
  EXPECT_FALSE(ik_->convert_cartesian_pose_to_joint_state_within_range(pose, ranges, result));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}