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

#include <queue>
#include <unordered_set>

namespace kinematics_interface_pinocchio
{
rclcpp::Logger LOGGER = rclcpp::get_logger("kinematics_interface_pinocchio");

bool KinematicsInterfacePinocchio::initialize(
  const std::string & robot_description,
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface,
  const std::string & param_namespace)
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
  // get verbose flag
  bool verbose = false;
  auto verbose_param = rclcpp::Parameter("verbose", verbose);
  if (parameters_interface->has_parameter(ns + "verbose"))
  {
    parameters_interface->get_parameter(ns + "verbose", verbose_param);
  }
  verbose = verbose_param.as_bool();

  // get alpha damping term
  auto alpha_param = rclcpp::Parameter("alpha", 0.000005);
  if (parameters_interface->has_parameter(ns + "alpha"))
  {
    parameters_interface->get_parameter(ns + "alpha", alpha_param);
  }
  alpha = alpha_param.as_double();

  // get end-effector name
  auto end_effector_name_param = rclcpp::Parameter("tip");
  if (parameters_interface->has_parameter(ns + "tip"))
  {
    parameters_interface->get_parameter(ns + "tip", end_effector_name_param);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Failed to find end effector name parameter [tip].");
    return false;
  }
  std::string end_effector_name = end_effector_name_param.as_string();

  // get model from parameter root name (if set)
  pinocchio::Model full_model;
  auto base_param = rclcpp::Parameter();
  if (parameters_interface->has_parameter(ns + "base"))
  {
    parameters_interface->get_parameter(ns + "base", base_param);
    root_name_ = base_param.as_string();
  }

  // load the model
  try
  {
    pinocchio::urdf::buildModelFromXML(robot_description_local, full_model, verbose, true);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(LOGGER, "Error parsing URDF to build Pinocchio model: %s", e.what());
    return false;
  }
  if (root_name_.empty())
  {
    // look for the first frame whose parent joint’s parent is universe (0)
    for (const auto & frame : full_model.frames)
    {
      if (frame.parent == 0 && frame.type == pinocchio::FrameType::BODY)  // BODY frame = link frame
      {
        root_name_ = frame.name;
        break;
      }
    }

    // Fallback if somehow not found
    if (root_name_.empty()) root_name_ = full_model.frames[0].name;  // usually "universe"
  }
  if (!full_model.existFrame(root_name_))
  {
    RCLCPP_ERROR(LOGGER, "failed to find robot root '%s' ", root_name_.c_str());
    return false;
  }
  if (!full_model.existFrame(end_effector_name))
  {
    RCLCPP_ERROR(LOGGER, "failed to find robot end effector '%s'", end_effector_name.c_str());
    return false;
  }

  // create reduced model by locking joints
  auto const get_descendants = [](const pinocchio::Model & model, pinocchio::JointIndex root)
    -> std::unordered_set<pinocchio::JointIndex>
  {
    std::unordered_set<pinocchio::JointIndex> descendants;
    std::queue<pinocchio::JointIndex> q;
    q.push(root);
    while (!q.empty())
    {
      auto j = q.front();
      q.pop();
      for (pinocchio::JointIndex k = 1; k < static_cast<pinocchio::JointIndex>(model.njoints); ++k)
      {
        if (model.parents[k] == j)
        {
          descendants.insert(k);
          q.push(k);
        }
      }
    }
    return descendants;
  };
  auto const get_chain_joints =
    [](
      const pinocchio::Model & model, pinocchio::JointIndex base_joint,
      pinocchio::JointIndex tool_joint) -> std::vector<pinocchio::JointIndex>
  {
    std::vector<pinocchio::JointIndex> chain;
    // Start from tool_joint and go upward to base_joint
    pinocchio::JointIndex current = tool_joint;
    while (current > 0)
    {
      chain.push_back(current);
      if (current == base_joint) break;
      current = model.parents[current];
    }
    std::reverse(chain.begin(), chain.end());
    // Remove base_joint itself → chain starts AFTER base link
    if (!chain.empty() && chain.front() == base_joint) chain.erase(chain.begin());
    return chain;
  };

  pinocchio::JointIndex base_joint_id = 0;  // default to universe
  if (root_name_ != "universe")
  {
    pinocchio::FrameIndex base_frame_id = full_model.getFrameId(root_name_);
    const pinocchio::Frame & base_frame = full_model.frames[base_frame_id];
    base_joint_id = base_frame.parent;  // the joint to which this frame is attached
  }

  pinocchio::FrameIndex end_effector_frame_id = full_model.getFrameId(end_effector_name);
  const pinocchio::Frame & end_effector_frame = full_model.frames[end_effector_frame_id];
  pinocchio::JointIndex tool_joint_id =
    end_effector_frame.parent;  // the joint to which this frame is attached

  // Validate: is tool under base?
  auto base_descendants = get_descendants(full_model, base_joint_id);
  if (base_descendants.find(tool_joint_id) == base_descendants.end())
  {
    RCLCPP_WARN(
      LOGGER, "Tool '%s' is not a descendant of base '%s'", end_effector_name.c_str(),
      root_name_.c_str());
    auto tip_descendants = get_descendants(full_model, tool_joint_id);
    if (tip_descendants.find(base_joint_id) == tip_descendants.end())
    {
      RCLCPP_ERROR(
        LOGGER, "Base frame '%s' is also not a descendant of tip '%s' — cannot form a chain.",
        end_effector_name.c_str(), root_name_.c_str());
      return false;
    }
    else
    {
      std::swap(root_name_, end_effector_name);
      RCLCPP_WARN(LOGGER, "Swapping tool and base frame");
    }
  }

  // Get joints in the subchain
  auto chain_joints = get_chain_joints(full_model, base_joint_id, tool_joint_id);
  RCLCPP_INFO(
    LOGGER, "Found chain from '%s' to tool frame '%s' with %zu joint(s)", root_name_.c_str(),
    end_effector_name.c_str(), chain_joints.size());
  std::unordered_set<pinocchio::JointIndex> chain_set(chain_joints.begin(), chain_joints.end());

  // Build list of joints to lock
  std::vector<pinocchio::JointIndex> locked_joints;
  for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(full_model.njoints);
       ++jid)
  {
    if (chain_set.find(jid) == chain_set.end())
    {
      locked_joints.push_back(jid);
    }
  }
  Eigen::VectorXd q_fixed =
    Eigen::VectorXd::Zero(full_model.nq);  // actual value is not important for kinematics
  model_ = pinocchio::buildReducedModel(full_model, locked_joints, q_fixed);

  // allocate dynamic memory
  data_ = std::make_shared<pinocchio::Data>(model_);
  num_joints_ = static_cast<Eigen::Index>(model_.nq);
  q_.resize(num_joints_);
  I = Eigen::MatrixXd(num_joints_, num_joints_);
  I.setIdentity();
  jacobian_.resize(6, num_joints_);
  jacobian_inverse_.resize(num_joints_, 6);

  return true;
}

bool KinematicsInterfacePinocchio::convert_joint_deltas_to_cartesian_deltas(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos,
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & delta_theta, const std::string & link_name,
  Eigen::Matrix<double, 6, 1> & delta_x)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_joint_vector(delta_theta))
  {
    RCLCPP_ERROR(LOGGER, "Input verification failed in '%s'", FUNCTION_SIGNATURE);
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
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos,
  const Eigen::Matrix<double, 6, 1> & delta_x, const std::string & link_name,
  Eigen::Matrix<double, Eigen::Dynamic, 1> & delta_theta)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_joint_vector(delta_theta))
  {
    RCLCPP_ERROR(LOGGER, "Input verification failed in '%s'", FUNCTION_SIGNATURE);
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
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_jacobian(jacobian))
  {
    RCLCPP_ERROR(LOGGER, "Input verification failed in '%s'", FUNCTION_SIGNATURE);
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
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse)
{
  // verify inputs
  if (
    !verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name) ||
    !verify_jacobian_inverse(jacobian_inverse))
  {
    RCLCPP_ERROR(LOGGER, "Input verification failed in '%s'", FUNCTION_SIGNATURE);
    return false;
  }

  // get joint array
  q_ = joint_pos;

  // calculate Jacobian
  const auto ee_frame_id = model_.getFrameId(link_name);
  pinocchio::computeFrameJacobian(model_, *data_, q_, ee_frame_id, jacobian_);
  // damped inverse
  jacobian_inverse_ =
    (jacobian_.transpose() * jacobian_ + alpha * I).inverse() * jacobian_.transpose();

  // TODO(anyone): fix sizes of jacobian inverse
  jacobian_inverse = jacobian_inverse_;

  return true;
}

bool KinematicsInterfacePinocchio::calculate_link_transform(
  const Eigen::Matrix<double, Eigen::Dynamic, 1> & joint_pos, const std::string & link_name,
  Eigen::Isometry3d & transform)
{
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) || !verify_link_name(link_name))
  {
    RCLCPP_ERROR(
      LOGGER, "Verification failed: %s",
      !verify_initialized()             ? "Not initialized"
      : !verify_joint_vector(joint_pos) ? "Invalid joint vector"
      : !verify_link_name(link_name)    ? "Invalid link name"
                                        : "Unknown error");
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

bool KinematicsInterfacePinocchio::verify_link_name(const std::string & link_name)
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
      LOGGER, "The link %s was not found in the robot chain. Available links are: %s",
      link_name.c_str(), links.c_str());
    return false;
  }
  return true;
}

bool KinematicsInterfacePinocchio::verify_joint_vector(const Eigen::VectorXd & joint_vector)
{
  if (joint_vector.size() != num_joints_)
  {
    RCLCPP_ERROR(
      LOGGER, "Invalid joint vector size (%zu). Expected size is %zu.", joint_vector.size(),
      num_joints_);
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
      LOGGER,
      "The Pinocchio kinematics plugin was not initialized. Ensure you called the initialize "
      "method.");
    return false;
  }
  return true;
}

bool KinematicsInterfacePinocchio::verify_jacobian(
  const Eigen::Matrix<double, 6, Eigen::Dynamic> & jacobian)
{
  if (jacobian.rows() != jacobian_.rows() || jacobian.cols() != jacobian_.cols())
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the jacobian (%zu, %zu) does not match the required size of (%zu, %zu)",
      jacobian.rows(), jacobian.cols(), jacobian_.rows(), jacobian_.cols());
    return false;
  }
  return true;
}

bool KinematicsInterfacePinocchio::verify_jacobian_inverse(
  const Eigen::Matrix<double, Eigen::Dynamic, 6> & jacobian_inverse)
{
  if (jacobian_inverse.rows() != jacobian_.cols() || jacobian_inverse.cols() != jacobian_.rows())
  {
    RCLCPP_ERROR(
      LOGGER, "The size of the jacobian (%zu, %zu) does not match the required size of (%zu, %zu)",
      jacobian_inverse.rows(), jacobian_inverse.cols(), jacobian_.cols(), jacobian_.rows());
    return false;
  }
  return true;
}

bool KinematicsInterfacePinocchio::verify_period(const double dt)
{
  if (dt < 0)
  {
    RCLCPP_ERROR(LOGGER, "The period (%f) must be a non-negative number", dt);
    return false;
  }
  return true;
}

}  // namespace kinematics_interface_pinocchio

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kinematics_interface_pinocchio::KinematicsInterfacePinocchio,
  kinematics_interface::KinematicsInterface)
