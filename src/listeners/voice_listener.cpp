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

#include <beine_dienen_legs/listeners/voice_listener.hpp>

#include <memory>
#include <string>

namespace beine_dienen_legs
{

using namespace std::chrono_literals;

VoiceListener::VoiceListener(rclcpp::Node::SharedPtr node, int listen_port)
: node(node),
  listener(std::make_shared<musen::StringListener>(listen_port)),
  command("")
{
}

VoiceListener::~VoiceListener()
{
  disconnect();
}

bool VoiceListener::connect()
{
  return listener->connect();
}

bool VoiceListener::disconnect()
{
  return listener->disconnect();
}

void VoiceListener::listen_process()
{
  auto message = listener->receive(32);

  if (message.size() > 0) {
    command = message;
  }
}

}  // namespace beine_dienen_legs
