// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <beine_dienen_legs/bridge.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

using Bridge = beine_dienen_legs::Bridge;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Initialize the node
  auto node = std::make_shared<rclcpp::Node>("dienen_legs_bridge");

  // Initialize the bridge
  std::shared_ptr<Bridge> bridge;
  if (argc > 2) {
    bridge = std::make_shared<Bridge>(node, atoi(argv[1]), atoi(argv[2]));
  } else if (argc > 1) {
    bridge = std::make_shared<Bridge>(node, atoi(argv[1]));
  } else {
    bridge = std::make_shared<Bridge>(node);
  }

  if (bridge->connect()) {
    rclcpp::spin(bridge->get_node());
  } else {
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
