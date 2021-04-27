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

#ifndef BEINE_DIENEN_LEGS__BRIDGE_HPP_
#define BEINE_DIENEN_LEGS__BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "./listeners/leg_listener.hpp"
#include "./listeners/voice_listener.hpp"

namespace beine_dienen_legs
{

class Bridge
{
public:
  Bridge(std::string node_name, int leg_port, int voice_port);

  bool connect();
  bool disconnect();

  rclcpp::Node::SharedPtr get_node();

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::TimerBase::SharedPtr listen_timer;

  std::shared_ptr<LegListener> leg_listener;
  std::shared_ptr<VoiceListener> voice_listener;
};

}  // namespace beine_dienen_legs

#endif  // BEINE_DIENEN_LEGS__BRIDGE_HPP_
