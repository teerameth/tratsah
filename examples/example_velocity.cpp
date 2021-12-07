// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("velocity_test_node");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;

  // Setup dimension
  std::vector<double> vec1 = { 1, 1, 1, 1};
  commands.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  commands.layout.dim[0].size = vec1.size();
  commands.layout.dim[0].stride = 4;
  commands.layout.dim[0].label = "x"; // Label
  // commands.data.push_back(0);
  // commands.data.push_back(0);
  commands.data.clear();
  commands.data.insert(commands.data.end(), vec1.begin(), vec1.end());

  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  // commands.data[0] = 1;
  // commands.data[1] = -1;
  publisher->publish(commands);
  std::this_thread::sleep_for(3s);

  vec1 = { -1, -1, -1, -1};
  commands.data.clear();
  commands.data.insert(commands.data.end(), vec1.begin(), vec1.end());
  // commands.data[0] = -1;
  // commands.data[1] = 1;
  publisher->publish(commands);
  std::this_thread::sleep_for(3s);

  // commands.data[0] = 0;
  // commands.data[1] = 0;
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);
  rclcpp::shutdown();

  return 0;
}
