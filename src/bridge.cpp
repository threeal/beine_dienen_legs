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

#include <bein_bridge/bridge.hpp>

#include <memory>
#include <string>

namespace bein_bridge
{

using namespace std::chrono_literals;

Bridge::Bridge(std::string node_name, int leg_port, int voice_port)
{
  // Initialize the node
  {
    node = std::make_shared<rclcpp::Node>(node_name);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Node initialized with name " << node->get_name() << "!");

    // Initialize the leg listener
    {
      leg_listener = std::make_shared<LegListener>(node, leg_port);

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Leg listener initialized on port " << leg_port << "!");
    }

    // Initialize the voice listener
    {
      voice_listener = std::make_shared<VoiceListener>(node, voice_port);

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Voice listener initialized on port " << voice_port << "!");
    }

    // Initialize the listen timer
    {
      listen_timer = node->create_wall_timer(
        10ms, [this]() {
          leg_listener->listen_process();
          voice_listener->listen_process();
        }
      );

      listen_timer->cancel();
    }
  }
}

bool Bridge::connect()
{
  if (!leg_listener->connect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect the leg listener!");
    return false;
  }

  if (!voice_listener->connect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect the voice listener!");
    return false;
  }

  listen_timer->reset();

  return true;
}

bool Bridge::disconnect()
{
  if (!leg_listener->disconnect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to disconnect the leg listener!");
    return false;
  }

  if (!voice_listener->disconnect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to disconnect the voice listener!");
    return false;
  }

  listen_timer->cancel();

  return true;
}

rclcpp::Node::SharedPtr Bridge::get_node()
{
  return node;
}

}  // namespace bein_bridge
