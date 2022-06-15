#include <gmock/gmock.h>
#include <memory>
#include "kinematics_interface/kinematics_interface_base.hpp"
#include "pluginlib/class_loader.hpp"
#include "ros2_control_test_assets/descriptions.hpp"


class TestKDLPlugin : public ::testing::Test {
public:
    std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>> ik_loader_;
    std::shared_ptr<kinematics_interface::KinematicsBaseClass> ik_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
    std::string end_effector = "link2";

    void SetUp() {
        // init ros
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");

        std::string plugin_name = "kdl_plugin/KDLKinematics";
        ik_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsBaseClass>>(
                "kdl_plugin", "kinematics_interface::KinematicsBaseClass");
        ik_ = std::unique_ptr<kinematics_interface::KinematicsBaseClass>(
                ik_loader_->createUnmanagedInstance(plugin_name));
    }

    void TearDown() {
        // shutdown ros
        rclcpp::shutdown();
    }

    void loadURDFParameter() {
        auto urdf = std::string(ros2_control_test_assets::urdf_head) + std::string(ros2_control_test_assets::urdf_tail);
        rclcpp::Parameter param("robot_description", urdf);
        node->declare_parameter("robot_description", "");
        node->set_parameter(param);
    }

    void loadAlphaParameter() {
        rclcpp::Parameter param("alpha", 0.005);
        node->declare_parameter("alpha", 0.005);
        node->set_parameter(param);
    }

};


TEST_F(TestKDLPlugin, KDL_plugin_function) {
    std::vector<double> pos = {0, 0};
    std::vector<double> vel = {0, 0};
    // load robot description and alpha to parameter server
    loadURDFParameter();
    loadAlphaParameter();

    ASSERT_TRUE(ik_->initialize(node, end_effector));
    ASSERT_TRUE(ik_->update_robot_state(pos, vel));

    // calculate end effector transform
    std::vector<double> end_effector_transform(3 + 9);
    ASSERT_TRUE(ik_->calculate_segment_transform(end_effector_transform, end_effector));

    // convert cartesian delta to joint delta
    std::vector<double> delta_x = {0, 0, 1, 0, 0, 0};
    std::vector<double> delta_theta(6);
    ASSERT_TRUE(ik_->convert_cartesian_deltas_to_joint_deltas(delta_x, delta_theta));

    // convert joint delta to cartesian delta
    std::vector<double> delta_x_est(6);
    ASSERT_TRUE(ik_->convert_joint_deltas_to_cartesian_deltas(delta_theta, delta_x_est));

}

TEST_F(TestKDLPlugin, KDL_plugin_no_robot_description) {
    loadAlphaParameter();

    ASSERT_FALSE(ik_->initialize(node, end_effector));

}