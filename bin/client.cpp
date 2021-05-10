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

#include <argparse/argparse.hpp>
#include <beine_dienen_legs/client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("client", "0.1.0");

  beine_dienen_legs::Client::Options options;

  program.add_argument("--legs-prefix")
  .help("prefix name for legs's topics and services")
  .action(
    [&](const std::string & value) {
      options.legs_prefix = value;
    });

  program.add_argument("--legs-port")
  .help("port number for legs communication")
  .action(
    [&](const std::string & value) {
      options.legs_port = stoi(value);
    });

  program.add_argument("--voice-port")
  .help("port number for voice communication")
  .action(
    [&](const std::string & value) {
      options.voice_port = stoi(value);
    });

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error & err) {
    std::cout << err.what() << std::endl;
    std::cout << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("client");
  auto client = std::make_shared<beine_dienen_legs::Client>(node, options);

  if (client->connect()) {
    rclcpp::spin(node);
  } else {
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
