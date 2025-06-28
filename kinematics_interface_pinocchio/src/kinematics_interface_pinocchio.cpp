// Copyright (c) 2024, Saif Sidhik.
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

#include "kinematics_interface_pinocchio/kinematics_interface_pinocchio.hpp"

namespace kinematics_interface_pinocchio
{
rclcpp::Logger LOGGER = rclcpp::get_logger("kinematics_interface_pinocchio");

bool KinematicsInterfacePinocchio::initialize(
    const std::string& robot_description,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
    const std::string& param_namespace
)
{
    // track initialization plugin
    initialized = true;

    // get parameters
    std::string ns = !param_namespace.empty() ? param_namespace + "." : "";

    std::string robot_description_local;
    if (robot_description.empty())
    {
        // If the robot_description input argument is empty, try to get the
        // robot_description from the node's parameters.
        auto robot_param = rclcpp::Parameter();
        if (!parameters_interface->get_parameter("robot_description", robot_param))
        {
            RCLCPP_ERROR(LOGGER, "parameter robot_description not set in kinematics_interface_pinocchio");
            return false;
        }
        robot_description_local = robot_param.as_string();
    }
    else
    {
        robot_description_local = robot_description;
    }
    // get alpha damping term
    auto alpha_param = rclcpp::Parameter("alpha", 0.000005);
    if (parameters_interface->has_parameter("alpha"))
    {
        parameters_interface->get_parameter("alpha", alpha_param);
    }
    alpha = alpha_param.as_double();

    // get root name
    auto base_param = rclcpp::Parameter();
    if (parameters_interface->has_parameter("base"))
    {
        parameters_interface->get_parameter("base", base_param);
        root_name_ = base_param.as_string();
    }
    else
    {
        root_name_ = model_.frames[0].name;
    }
    // TODO: only handling fixed base now
    model_ = pinocchio::urdf::buildModelFromXML(robot_description_local, /*root_name_,*/ model_, true);

    // allocate dynamics memory
    data_ = std::make_shared<pinocchio::Data>(model_);
    num_joints_ = model_.nq; // TODO: handle floating base
    q_.resize(num_joints_);
    I = Eigen::MatrixXd(num_joints_, num_joints_);
    I.setIdentity();
    jacobian_.resize(6, num_joints_);
    jacobian_inverse_.resize(num_joints_, 6);

    return true;
}

bool KinematicsInterfacePinocchio::convert_joint_deltas_to_cartesian_deltas(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& joint_pos,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& delta_theta, const std::string& link_name,
    Eigen::Matrix<double, 6, 1>& delta_x
)
{
    // verify inputs
    if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
        !verify_joint_vector(delta_theta))
    {
        return false;
    }

    // get joint array
    q_ = joint_pos;

    // calculate Jacobian
    const auto ee_frame_id = model_.getFrameId(link_name);
    pinocchio::computeFrameJacobian(model_, *data_, q_, ee_frame_id, jacobian_);
    delta_x = jacobian_ * delta_theta;

    return true;
}

bool KinematicsInterfacePinocchio::convert_cartesian_deltas_to_joint_deltas(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& joint_pos, const Eigen::Matrix<double, 6, 1>& delta_x,
    const std::string& link_name, Eigen::Matrix<double, Eigen::Dynamic, 1>& delta_theta
)
{
    // verify inputs
    if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
        !verify_joint_vector(delta_theta))
    {
        return false;
    }

    // calculate Jacobian inverse
    if (!calculate_jacobian_inverse(joint_pos, link_name, jacobian_inverse_))
    {
        return false;
    }

    delta_theta = jacobian_inverse_ * delta_x;

    return true;
}

bool KinematicsInterfacePinocchio::calculate_jacobian(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& joint_pos, const std::string& link_name,
    Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian
)
{
    // verify inputs
    if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
        !verify_jacobian(jacobian))
    {
        return false;
    }

    // get joint array
    q_ = joint_pos;

    // calculate Jacobian
    const auto ee_frame_id = model_.getFrameId(link_name);
    pinocchio::computeFrameJacobian(model_, *data_, q_, ee_frame_id, jacobian_);
    jacobian = jacobian_;

    return true;
}

bool KinematicsInterfacePinocchio::calculate_jacobian_inverse(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& joint_pos, const std::string& link_name,
    Eigen::Matrix<double, Eigen::Dynamic, 6>& jacobian_inverse
)
{
    // verify inputs
    if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
        !verify_jacobian_inverse(jacobian_inverse))
    {
        return false;
    }

    // get joint array
    q_ = joint_pos;

    // calculate Jacobian
    const auto ee_frame_id = model_.getFrameId(link_name);
    pinocchio::computeFrameJacobian(model_, *data_, q_, ee_frame_id, jacobian_);
    // damped inverse
    jacobian_inverse_ = (jacobian_.transpose() * jacobian_ + alpha * I).inverse() * jacobian_.transpose();

    jacobian_inverse = jacobian_inverse_;

    return true;
}

bool KinematicsInterfacePinocchio::calculate_link_transform(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& joint_pos, const std::string& link_name,
    Eigen::Isometry3d& transform
)
{
    // verify inputs
    if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name))
    {
        RCLCPP_ERROR(
            LOGGER, "Verification failed: %s",
            !verify_initialized()             ? "Not initialized"
            : !verify_joint_vector(joint_pos) ? "Invalid joint vector"
            : !verify_link_name(link_name)    ? "Invalid link name"
                                              : "Unknown error"
        );
        return false;
    }

    // get joint array
    q_ = joint_pos;

    // reset transform_vec
    transform.setIdentity();

    // special case: since the root is not in the robot tree, need to return identity transform
    if (link_name == root_name_)
    {
        return true;
    }

    // calculate Jacobian
    const auto ee_frame_id = model_.getFrameId(link_name);

    // Perform forward kinematics and get a transform.
    pinocchio::framesForwardKinematics(model_, *data_, q_);
    frame_tf_ = data_->oMf[ee_frame_id].toHomogeneousMatrix();

    transform.linear() = frame_tf_.block<3, 3>(0, 0);
    transform.translation() = frame_tf_.block<3, 1>(0, 3);
    return true;
}

bool KinematicsInterfacePinocchio::verify_link_name(const std::string& link_name)
{
    if (link_name == root_name_)
    {
        return true;
    }
    if (!model_.existBodyName(link_name))
    {
        std::string links;
        for (size_t i = 0; i < model_.frames.size(); ++i)
        {
            links += "\n" + model_.frames[i].name;
        }
        RCLCPP_ERROR(
            LOGGER, "The link %s was not found in the robot chain. Available links are: %s", link_name.c_str(),
            links.c_str()
        );
        return false;
    }
    return true;
}

bool KinematicsInterfacePinocchio::verify_joint_vector(const Eigen::VectorXd& joint_vector)
{
    if (static_cast<size_t>(joint_vector.size()) != num_joints_)
    {
        RCLCPP_ERROR(
            LOGGER, "Invalid joint vector size (%zu). Expected size is %zu.", joint_vector.size(), num_joints_
        );
        return false;
    }
    return true;
}

bool KinematicsInterfacePinocchio::verify_initialized()
{
    // check if interface is initialized
    if (!initialized)
    {
        RCLCPP_ERROR(
            LOGGER, "The Pinocchio kinematics plugin was not initialized. Ensure you called the initialize method."
        );
        return false;
    }
    return true;
}

bool KinematicsInterfacePinocchio::verify_jacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian)
{
    if (jacobian.rows() != jacobian_.rows() || jacobian.cols() != jacobian_.cols())
    {
        RCLCPP_ERROR(
            LOGGER, "The size of the jacobian (%zu, %zu) does not match the required size of (%zu, %zu)",
            jacobian.rows(), jacobian.cols(), jacobian_.rows(), jacobian_.cols()
        );
        return false;
    }
    return true;
}

bool KinematicsInterfacePinocchio::verify_jacobian_inverse(
  const Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse)
{
  if (
    jacobian_inverse.rows() != jacobian_.cols() || jacobian_inverse.cols() != jacobian_.rows())
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the jacobian (%zu, %zu) does not match the required size of (%zu, %zu)",
      jacobian_inverse.rows(), jacobian_inverse.cols(), jacobian_.cols(), jacobian_.rows());
    return false;
  }
  return true;
}

} // namespace kinematics_interface_pinocchio

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kinematics_interface_pinocchio::KinematicsInterfacePinocchio, kinematics_interface::KinematicsInterface
)
