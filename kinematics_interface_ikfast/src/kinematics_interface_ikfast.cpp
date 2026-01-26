#include "kinematics_interface_ikfast/kinematics_interface_ikfast.hpp"
#include <cmath>
#include "kinematics_interface_ikfast/ikfast.h"

namespace kinematics_interface_ikfast
{
rclcpp::Logger LOGGER = rclcpp::get_logger("kinematics_interface_ikfast");

const int MAX_IK_SOLUTIONS = 8;

bool KinematicsInterfaceIKFast::initialize(
  const std::string & robot_description, //unused but lets keep for now
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

  I_ = Eigen::MatrixXd::Identity(num_joints_, num_joints_); // set this for Jacobian calculation

  initialized_ = true;
  return true;
}

// Forward Kinematics
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

  compute_fk(vjoints.data(), eetrans, eerot);
  RCLCPP_INFO(LOGGER, "vjoints.data = %p", static_cast<const void*>(vjoints.data()));

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

bool KinematicsInterfaceIKFast::convert_cartesian_pose_to_closest_joint_state(
  const Eigen::Isometry3d & pose, const std::vector<double> & current_joint_state,
  std::vector<double> & joint_state)
{
  std::vector<std::vector<double>> all_states;
  if (!convert_cartesian_pose_to_all_possible_joint_states(pose, all_states)) return false;
  if (all_states.empty()) return false;

  double min_sum = 1e10;
  std::vector<double> best;
  for (const auto& sol : all_states) {
    double sum = 0.0;
    for (size_t j = 0; j < static_cast<size_t>(num_joints_); ++j) {
      double diff = sol[j] - current_joint_state[j];
      while (diff > M_PI) diff -= 2 * M_PI;
      while (diff < -M_PI) diff += 2 * M_PI;
      sum += std::fabs(diff);
    }
    if (sum < min_sum) {
      min_sum = sum;
      best = sol;
    }
  }
  joint_state = best;
  return true;
}

bool KinematicsInterfaceIKFast::convert_cartesian_pose_to_joint_state_within_range(
  const Eigen::Isometry3d & pose, const std::vector<std::pair<double, double>> & joint_ranges,
  std::vector<double> & joint_state)
{
  std::vector<std::vector<double>> all_states;
  if (!convert_cartesian_pose_to_all_possible_joint_states(pose, all_states)) return false;

  const double TWO_PI = 2.0 * M_PI;

  for (const auto& sol : all_states) {
    bool all_joints_valid = true;
    std::vector<double> adjusted_sol(num_joints_);

    for (size_t j = 0; j < static_cast<size_t>(num_joints_); ++j) {
      double low = joint_ranges[j].first;
      double high = joint_ranges[j].second;
      double s = sol[j];

      // Case 1 : No constraint (NaN)
      if (std::isnan(low) || std::isnan(high)) {
        adjusted_sol[j] = s;
        continue;
      }

      // Case 2: Exact value constraint (low == high)
      if (std::abs(low - high) < 1e-6) {
        double diff = std::fmod(s - low, TWO_PI);
        if (diff > M_PI) diff -= TWO_PI;
        if (diff < -M_PI) diff += TWO_PI;

        if (std::abs(diff) > 1e-6) {
          all_joints_valid = false;
          break;
        }
        adjusted_sol[j] = low;
      }
      // Case 3: Range constraint
      else {
        double shifted_s = s;

        // Bring it up if too low
        while (shifted_s < low) shifted_s += TWO_PI;
        // Bring it down if too high
        while (shifted_s > high) shifted_s -= TWO_PI;
        if (shifted_s < low || shifted_s > high) {
          all_joints_valid = false;
          break;
        }
        adjusted_sol[j] = shifted_s;
      }
    }

    if (all_joints_valid) {
      joint_state = adjusted_sol;
      return true;
    }
  }
  return false;
}

bool KinematicsInterfaceIKFast::convert_cartesian_pose_to_all_possible_joint_states(
  const Eigen::Isometry3d & pose, std::vector<std::vector<double>> & joint_states)
{
  if (!verify_initialized()) return false;

  double eetrans[3];
  eetrans[0] = pose.translation().x();
  eetrans[1] = pose.translation().y();
  eetrans[2] = pose.translation().z();

  double eerot[9];
  Eigen::Matrix3d rot = pose.rotation();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      eerot[i * 3 + j] = rot(i, j);
    }
  }

  ikfast::IkSolutionList<double> solutions;
  compute_ik(eetrans, eerot, nullptr, (void*)&solutions);

  joint_states.clear();
  for (size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
    std::vector<double> joints(num_joints_);
    const ikfast::IkSolutionBase<double>& sol = solutions.GetSolution(i);
    sol.GetSolution(&joints[0], nullptr);
    bool valid = true;
    for (double j : joints) {
      if (std::isnan(j)) {
        valid = false;
        break;
      }
    }
    if (valid) {
      joint_states.push_back(joints);
    }
  }
  return true;
}

bool KinematicsInterfaceIKFast::convert_joint_state_to_cartesian_pose(
  const std::vector<double> & joint_state, Eigen::Isometry3d & pose)
{
  Eigen::VectorXd joint_pos = Eigen::Map<const Eigen::VectorXd>(joint_state.data(), joint_state.size());
  return calculate_link_transform(joint_pos, end_effector_name_, pose);
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
