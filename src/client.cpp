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

#include <beine_dienen_legs/client.hpp>

#include <memory>

namespace beine_dienen_legs
{

using namespace std::chrono_literals;

Client::Client(rclcpp::Node::SharedPtr node, int legs_port, int voice_port)
{
  // Initialize the node
  this->node = node;

  // Initialize the listen timer
  {
    listen_timer = get_node()->create_wall_timer(
      10ms, [this]() {
        legs_listen_process();
        voice_listen_process();
      });

    listen_timer->cancel();
  }

  // Initialize the legs provider
  legs_provider = std::make_shared<beine_cpp::LegsProvider>(get_node());

  // Initialize the legs listener
  {
    legs_listener = std::make_shared<musen::StringListener>(legs_port);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Legs listener initialized on port " << legs_listener->get_port() << "!");
  }

  // Initialize the voice listener
  {
    voice_listener = std::make_shared<musen::StringListener>(voice_port);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Voice listener initialized on port " << voice_listener->get_port() << "!");
  }
}

bool Client::connect()
{
  if (!legs_listener->connect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect the legs listener!");
    return false;
  }

  if (!voice_listener->connect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect the voice listener!");
    return false;
  }

  listen_timer->reset();

  return true;
}

bool Client::disconnect()
{
  if (!legs_listener->disconnect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to disconnect the legs listener!");
    return false;
  }

  if (!voice_listener->disconnect()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to disconnect the voice listener!");
    return false;
  }

  listen_timer->cancel();

  return true;
}

rclcpp::Node::SharedPtr Client::get_node() const
{
  return node;
}

void Client::legs_listen_process()
{
  auto message = legs_listener->receive(64, ",");

  if (message.size() > 0) {
    try {
      size_t i = 0;

      // Update position data
      {
        beine_cpp::Position position;

        position.x = stod(message[i++]);
        position.y = stod(message[i++]);
        position.z = stod(message[i++]);

        legs_provider->set_position(position);
      }

      // Update orientation data
      {
        beine_cpp::Orientation orientation;

        orientation.x = stod(message[i++]);
        orientation.y = stod(message[i++]);
        orientation.z = stod(message[i++]);

        legs_provider->set_orientation(orientation);
      }

      // Update joints data
      {
        beine_cpp::Joints joints;

        joints.left_knee = stod(message[i++]);
        joints.left_ankle = stod(message[i++]);
        joints.right_knee = stod(message[i++]);
        joints.right_ankle = stod(message[i++]);

        legs_provider->set_joints(joints);
      }
    } catch (const std::out_of_range & err) {
      RCLCPP_WARN_STREAM(node->get_logger(), "Not all values are received! " << err.what());
    }
  }
}

void Client::voice_listen_process()
{
  auto message = voice_listener->receive(32);

  if (message.size() > 0) {
    legs_provider->set_command(message);
  }
}

}  // namespace beine_dienen_legs
