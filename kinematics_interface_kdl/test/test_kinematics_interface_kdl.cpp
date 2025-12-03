// Copyright (c) 2025, Austrian Institute of Technology.
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
/// \author: Christoph Froehlich

#include <gmock/gmock.h>

#include "kinematics_interface_tests/kinematics_interface_common_tests.hpp"

struct PluginKDL
{
  static std::string Name() { return "kinematics_interface_kdl/KinematicsInterfaceKDL"; }
  static void set_custom_node_parameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  {
    node->declare_parameter("alpha", 0.005);
  }
};

using KinematicsInterfaceKDLTestTypes = ::testing::Types<PluginKDL>;
INSTANTIATE_TYPED_TEST_SUITE_P(PluginTestKDL, TestPlugin, KinematicsInterfaceKDLTestTypes);
