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

#include <bein_bridge/voice_listener.hpp>

#include <memory>
#include <string>

namespace bein_bridge
{

using namespace std::chrono_literals;

VoiceListener::VoiceListener(std::string node_name, int listen_port)
: command("")
{
  // Initialize the node
  {
    node = std::make_shared<rclcpp::Node>(node_name);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Node initialized with name " << node->get_name() << "!");

    // Initialize the listen timer
    {
      listen_timer = node->create_wall_timer(
        10ms, [this]() {
          command = listener->receive(32);
        }
      );

      listen_timer->cancel();
    }
  }

  // Initialize the listener
  {
    listener = std::make_shared<housou::StringListener>(listen_port);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Listener initialized on port " << listener->get_port() << "!");
  }
}

VoiceListener::~VoiceListener()
{
  disconnect();
}

bool VoiceListener::connect()
{
  if (!listener->connect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect the listener!");
    return false;
  }

  listen_timer->reset();

  return true;
}

bool VoiceListener::disconnect()
{
  if (!listener->disconnect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to disconnect the listener!");
    return false;
  }

  listen_timer->cancel();

  return true;
}

rclcpp::Node::SharedPtr VoiceListener::get_node()
{
  return node;
}

}  // namespace bein_bridge
