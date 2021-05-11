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

#ifndef BEINE_DIENEN_LEGS__CLIENT_HPP_
#define BEINE_DIENEN_LEGS__CLIENT_HPP_

#include <beine_cpp/beine_cpp.hpp>
#include <musen/musen.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace beine_dienen_legs
{

class Client : public beine_cpp::LegsNode
{
public:
  struct Options : public virtual LegsNode::Options, public virtual beine_cpp::LegsProvider::Options
  {
    int legs_port;
    int voice_port;

    Options()
    : legs_port(3343),
      voice_port(6343)
    {
    }
  };

  explicit Client(rclcpp::Node::SharedPtr node, const Options & options);

  bool connect();
  bool disconnect();

private:
  void legs_listen_process();
  void voice_listen_process();

  rclcpp::TimerBase::SharedPtr listen_timer;

  std::shared_ptr<beine_cpp::LegsProvider> legs_provider;

  std::shared_ptr<musen::StringListener> legs_listener;
  std::shared_ptr<musen::StringListener> voice_listener;
};

}  // namespace beine_dienen_legs

#endif  // BEINE_DIENEN_LEGS__CLIENT_HPP_
