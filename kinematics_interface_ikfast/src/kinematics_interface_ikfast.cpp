#include "kinematics_interface_ikfast/kinematics_interface_ikfast.hpp"

namespace kinematics_interface_ikfast
{
rclcpp::Logger LOGGER = rclcpp::get_logger("kinematics_interface_ikfast");

bool KinematicsInterfaceIKFast::initialize(
  const std::string & robot_description,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & param_namespace)
{
  if (robot_description.empty()) return false;

  std::string ns = !param_namespace.empty() ? param_namespace + "." : "";

  // Parameters
  auto end_effector_name_param = rclcpp::Parameter("tip", "flange");
  parameters_interface->get_parameter(ns + "tip", end_effector_name_param);
  end_effector_name_ = end_effector_name_param.as_string();

  auto root_param = rclcpp::Parameter("base", "base_link");
  parameters_interface->get_parameter(ns + "base", root_param);
  root_name_ = root_param.as_string();

  auto alpha_param = rclcpp::Parameter("alpha", 0.000005);
  parameters_interface->get_parameter(ns + "alpha", alpha_param);
  alpha_ = alpha_param.as_double();

  num_joints_ = get_num_joints_internal();

  // Debug log
  RCLCPP_INFO(rclcpp::get_logger("kinematics_interface_ikfast"),
              "Plugin initialized with %d joints.", num_joints_);

  if (num_joints_ <= 0) {
      RCLCPP_ERROR(rclcpp::get_logger("kinematics_interface_ikfast"),
                   "Joint number is non-positive: %d", num_joints_);
      return false;
  }

  I_ = Eigen::MatrixXd::Identity(num_joints_, num_joints_);

  initialized_ = true;
  return true;
}

bool KinematicsInterfaceIKFast::calculate_link_transform(
  const Eigen::VectorXd & joint_pos,
  const std::string & link_name,
  Eigen::Isometry3d & transform)
{
  if (!verify_initialized() || !verify_joint_vector(joint_pos)) return false;

  if (link_name != end_effector_name_) {
    RCLCPP_WARN(LOGGER, "Requested link [%s] does not match tip [%s]", link_name.c_str(), end_effector_name_.c_str());
    return false;
  }

  std::vector<double> vjoints(num_joints_);
  Eigen::VectorXd::Map(&vjoints[0], num_joints_) = joint_pos;

  double eerot[9], eetrans[3];

  do_compute_fk(vjoints.data(), eetrans, eerot);

  Eigen::Matrix3d rotation;
  rotation << eerot[0], eerot[1], eerot[2],
              eerot[3], eerot[4], eerot[5],
              eerot[6], eerot[7], eerot[8];

  transform.setIdentity();
  transform.linear() = rotation;
  transform.translation() << eetrans[0], eetrans[1], eetrans[2];
  return true;
}

bool KinematicsInterfaceIKFast::calculate_jacobian(
  const Eigen::VectorXd & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_jacobian(jacobian)) return false;

  jacobian.setZero(6, num_joints_);
  Eigen::Isometry3d T_nominal, T_perturbed;

  if (!calculate_link_transform(joint_pos, link_name, T_nominal)) return false;

  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    Eigen::VectorXd q_perturbed = joint_pos;
    q_perturbed[i] += epsilon_;

    if (!calculate_link_transform(q_perturbed, link_name, T_perturbed)) return false;

    jacobian.block<3, 1>(0, i) = (T_perturbed.translation() - T_nominal.translation()) / epsilon_;
    Eigen::Matrix3d R_diff = T_perturbed.linear() * T_nominal.linear().transpose();
    Eigen::AngleAxisd angle_axis(R_diff);
    jacobian.block<3, 1>(3, i) = (angle_axis.axis() * angle_axis.angle()) / epsilon_;
  }
  return true;
}

bool KinematicsInterfaceIKFast::calculate_jacobian_inverse(
  const Eigen::VectorXd & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse)
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
  jacobian.resize(6, num_joints_);

  jacobian_inverse.resize(num_joints_, 6);

  if (!calculate_jacobian(joint_pos, link_name, jacobian) || !verify_jacobian_inverse(jacobian_inverse)) return false;

  Eigen::Matrix<double, 6, 6> A = jacobian * jacobian.transpose();
  A += (alpha_ * alpha_) * Eigen::Matrix<double, 6, 6>::Identity();
  jacobian_inverse = jacobian.transpose() * A.inverse();
  return true;
}

bool KinematicsInterfaceIKFast::convert_cartesian_deltas_to_joint_deltas(
  const Eigen::VectorXd & joint_pos, const Eigen::Matrix<double, 6, 1> & delta_x,
  const std::string & link_name, Eigen::VectorXd & delta_theta)
{
  Eigen::Matrix<double, Eigen::Dynamic, 6> J_inv;
  J_inv.resize(num_joints_, 6);
  delta_theta.resize(num_joints_);
  if (!calculate_jacobian_inverse(joint_pos, link_name, J_inv)) return false;
  delta_theta = J_inv * delta_x;
  return true;
}

bool KinematicsInterfaceIKFast::convert_joint_deltas_to_cartesian_deltas(
  const Eigen::VectorXd & joint_pos, const Eigen::VectorXd & delta_theta,
  const std::string & link_name, Eigen::Matrix<double, 6, 1> & delta_x)
{
  if (delta_theta.size() != num_joints_) return false;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J;
  J.resize(6, num_joints_);

  if (!calculate_jacobian(joint_pos, link_name, J)) return false;
  delta_x = J * delta_theta;
  return true;
}

bool KinematicsInterfaceIKFast::verify_initialized()
{
  // check if interface is initialized
  if (!initialized_)
  {
    RCLCPP_ERROR(
      LOGGER,
      "The IKFast kinematics plugin was not initialized. Ensure you called the initialize method.");
    return false;
  }
  return true;
}
bool KinematicsInterfaceIKFast::verify_joint_vector(const Eigen::VectorXd & joint_vector)
{
  if (joint_vector.size() != num_joints_) return false;
  return true;
}

bool KinematicsInterfaceIKFast::verify_jacobian(
  const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  if (jacobian.rows() != 6 || jacobian.cols() != num_joints_)
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the jacobian (%zu, %zu) does not match the required size of (%u, %u)",
      jacobian.rows(), jacobian.cols(), 6, num_joints_);
    return false;
  }
  return true;
}

bool KinematicsInterfaceIKFast::verify_jacobian_inverse(
  const Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian)
{
  if (jacobian.rows() != num_joints_ || jacobian.cols() != 6)
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the jacobian inverse (%zu, %zu) does not match the required size of (%u, %u)",
      jacobian.rows(), jacobian.cols(), num_joints_, 6);
    return false;
  }
  return true;
}

} // namespace kinematics_interface_ikfast
