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

#include <bein_bridge/bein_bridge.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: ros2 run bein_bridge bridge <leg_port> <voice_port>" << std::endl;
    return 1;
  }

  int leg_port = atoi(argv[1]);
  int voice_port = atoi(argv[2]);

  rclcpp::init(argc, argv);

  auto bridge = std::make_shared<bein_bridge::Bridge>(
    "bein", leg_port, voice_port
  );

  if (bridge->connect()) {
    rclcpp::spin(bridge->get_node());
  } else {
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
