#ifndef KINEMATICS_INTERFACE_IKFAST__KINEMATICS_INTERFACE_IKFAST_HPP_
#define KINEMATICS_INTERFACE_IKFAST__KINEMATICS_INTERFACE_IKFAST_HPP_

#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "kinematics_interface/kinematics_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kinematics_interface_ikfast
{
/**
 * @brief Generic IKFast Kinematics Wrapper
 * This class implements the KinematicsInterface for robots with IKFast-generated kinematics.
 */
class KinematicsInterfaceIKFast : public kinematics_interface::KinematicsInterface
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

  // Virtual function to get number of joints from IKFast
  virtual int get_num_joints_internal() = 0;
  virtual void do_compute_fk(const double* j, double* etrans, double* erot) = 0;
  virtual void do_compute_ik(const double* etrans, const double* erot, const double* free, void* solutions) = 0;

private:
  // verification methods
  bool verify_initialized();
  bool verify_joint_vector(const Eigen::VectorXd & joint_vector);
  bool verify_jacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian);
  bool verify_jacobian_inverse(const Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian);

  bool initialized_ = false;
  int num_joints_;
  std::string root_name_;
  std::string end_effector_name_;
  const double epsilon_ = 1e-6;

  double alpha_;  // damping term for Jacobian inverse
  Eigen::MatrixXd I_;
};

}  // namespace kinematics_interface_ikfast

#endif  // KINEMATICS_INTERFACE_IKFAST__KINEMATICS_INTERFACE_IKFAST_HPP_