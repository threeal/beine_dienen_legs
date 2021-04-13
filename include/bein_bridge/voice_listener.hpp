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

#ifndef BEIN_BRIDGE__VOICE_LISTENER_HPP_
#define BEIN_BRIDGE__VOICE_LISTENER_HPP_

#include <housou/housou.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace bein_bridge
{

class VoiceListener
{
public:
  VoiceListener(std::string node_name, int listen_port);
  ~VoiceListener();

  bool connect();
  bool disconnect();

  rclcpp::Node::SharedPtr get_node();

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::TimerBase::SharedPtr listen_timer;

  std::shared_ptr<housou::StringListener> listener;

  std::string command;
};

}  // namespace bein_bridge

#endif  // BEIN_BRIDGE__VOICE_LISTENER_HPP_
