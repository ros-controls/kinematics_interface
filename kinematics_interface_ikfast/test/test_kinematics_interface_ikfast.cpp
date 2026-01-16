#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <kinematics_interface/kinematics_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

class IKFastPluginTest : public ::testing::Test {
protected:
  void SetUp() override {
    loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");
  }
  std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
};

TEST_F(IKFastPluginTest, FullInterfaceValidation) {
  try {
    auto instance = loader_->createSharedInstance("fanuc_ikfast/FanucKinematics");
    ASSERT_NE(instance, nullptr);

    auto node = std::make_shared<rclcpp::Node>("test_node");
    node->declare_parameter("tip", "flange");
    node->declare_parameter("base", "base_link");
    node->declare_parameter("alpha", 1e-6);

    ASSERT_TRUE(instance->initialize("dummy", node->get_node_parameters_interface(), ""));

    // Test variables
    int num_joints = 6; // Fanuc
    Eigen::VectorXd joints = Eigen::VectorXd::Zero(num_joints);
    joints << 0.1, -0.2, 0.15, 0.0, 0.5, 0.1; // Random configuration

    // 1. FORWARD KINEMATICS TEST
    Eigen::Isometry3d transform;
    EXPECT_TRUE(instance->calculate_link_transform(joints, "flange", transform));
    std::cout << "[CHECK] FK Position: " << transform.translation().transpose() << std::endl;

    // 2. JACOBIAN TEST
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
    jacobian.resize(6, num_joints);
    EXPECT_TRUE(instance->calculate_jacobian(joints, "flange", jacobian));
    EXPECT_EQ(jacobian.rows(), 6);
    EXPECT_EQ(jacobian.cols(), num_joints);
    std::cout << "[CHECK] Jacobian calculated (6x" << num_joints << ")" << std::endl;

    // 3. INVERSE JACOBIAN TEST (Damped Least Squares)
    Eigen::Matrix<double, Eigen::Dynamic, 6> inv_jacobian;
    inv_jacobian.resize(num_joints, 6);
    EXPECT_TRUE(instance->calculate_jacobian_inverse(joints, "flange", inv_jacobian));
    std::cout << "[CHECK] Inverse Jacobian calculated." << std::endl;

    // 4. CARTESIAN DELTA TO JOINT DELTA (IK Velocity)
    Eigen::Matrix<double, 6, 1> delta_x;
    delta_x << 0.01, 0, 0, 0, 0, 0; // X 1cm movement
    Eigen::VectorXd delta_theta;
    EXPECT_TRUE(instance->convert_cartesian_deltas_to_joint_deltas(joints, delta_x, "flange", delta_theta));
    EXPECT_EQ(delta_theta.size(), num_joints);
    std::cout << "[CHECK] Cartesian Delta -> Joint Delta: " << delta_theta.transpose() << std::endl;

    // 5. JOINT DELTA TO CARTESIAN DELTA (FK Velocity)
    Eigen::Matrix<double, 6, 1> output_delta_x;
    EXPECT_TRUE(instance->convert_joint_deltas_to_cartesian_deltas(joints, delta_theta, "flange", output_delta_x));

    // J * J_inv * dx ~= dx olmalı
    for(int i=0; i<3; ++i) {
        EXPECT_NEAR(delta_x(i), output_delta_x(i), 1e-4);
    }
    std::cout << "[CHECK] Velocity Roundtrip Validation Successful." << std::endl;

  } catch (const pluginlib::PluginlibException& ex) {
    FAIL() << "Plugin error: " << ex.what();
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}