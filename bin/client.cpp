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
#include <rclcpp/rclcpp.hpp>

#include <memory>

using Client = beine_dienen_legs::Client;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Initialize the node
  auto node = std::make_shared<rclcpp::Node>("dienen_legs_client");

  // Initialize the bridge
  std::shared_ptr<Client> client;
  if (argc > 2) {
    client = std::make_shared<Client>(node, atoi(argv[1]), atoi(argv[2]));
  } else if (argc > 1) {
    client = std::make_shared<Client>(node, atoi(argv[1]));
  } else {
    client = std::make_shared<Client>(node);
  }

  if (client->connect()) {
    rclcpp::spin(node);
  } else {
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
